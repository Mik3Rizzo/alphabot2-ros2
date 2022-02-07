import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from alphabot2_interfaces.msg import Obstacle

import RPi.GPIO as GPIO

"""
AlphaBot2 is equipped with Toshiba TB6612FNG IC dual motor driver.
Every motor has 3 inputs controlled by the Raspberry Pi:
  - xIN1 = digital, controls direction
  - xIN2 = digital, controls direction
  - PWMx = PWM, controls motor's speed, up to 100kHz
REF: https://www.instructables.com/Driving-Small-Motors-With-the-TB6612FNG/
"""

# Toshiba TB6612FNG / Raspberry Pi GPIO pin mapping
PWMA_PIN = 6    # L-motor speed PWM
AIN1_PIN = 12   # L-motor backward
AIN2_PIN = 13   # L-motor forward
PWMB_PIN = 26   # R-motor speed PWM
BIN1_PIN = 20   # R-motor backward
BIN2_PIN = 21   # R-motor forward

DEFAULT_MOTORS_PWM_DUTY_CYCLE = 50  # L/R-motor default PWM duty cycle
DEFAULT_MOTORS_PWM_FREQUENCY = 500  # L/R-motor default PWM frequency (experimental)

MOTOR_BRAKE_TIMEOUT_SEC = 2         # after 2 secs of non received cmd_vel msgs, motors will be braked
SPIN_TIMER_PERIOD_SEC = 0.025       # timer callback period (40 Hz >> 1.5cm run in one period at 0.6 m/s)
MAX_BRAKING_DURATION_SEC = 0.10     # max duration of the opposite direction motion for braking

WHEELS_DIST_M = 0.085                       # 85 mm, distance between wheels
WHEEL_RADIUS_M = 0.021                      # 21 mm, wheel radius
MAX_MOTOR_RPM = 750                         # max AlphaBot2 wheel RPM (80 ms for single revolution) if li-ion supplied
                                            # (400 RPM for USB supply)
RPM_TO_RAD_PER_SEC = 2 * math.pi / 60       # conversion factor from RPM to rad/s

OBSTACLES_TOPIC = "obstacles"
CMD_VEL_TOPIC = "cmd_vel"


def gpio_init():
    """
    Function that initializes GPIO.
    """
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)


def gpio_close():
    """
    Function that closes GPIO.
    """
    GPIO.cleanup()


def clip(value, minimum, maximum):
    """
    Function that clips a value between a minimum and a maximum.
    :param value: value to clip
    :param minimum: minimum value
    :param maximum: maximum value
    :return: clipped value
    """
    if value < minimum:
        return minimum
    elif value > maximum:
        return maximum
    return value


class Motor:
    """
    Class that represents a single AlphaBot2 motor.
    """
    def __init__(self, forward_pin, backward_pin, speed_pwm_pin):

        # Setup pin numbers
        self.forward_pin = forward_pin
        self.backward_pin = backward_pin
        self.speed_pwm_pin = speed_pwm_pin

        # Setup pins as OUTPUT
        GPIO.setup(self.forward_pin, GPIO.OUT)
        GPIO.setup(self.backward_pin, GPIO.OUT)
        GPIO.setup(self.speed_pwm_pin, GPIO.OUT)

        # Initialize the PWM pin
        self.speed_pwm = GPIO.PWM(self.speed_pwm_pin, DEFAULT_MOTORS_PWM_FREQUENCY)
        self.speed_pwm.start(DEFAULT_MOTORS_PWM_DUTY_CYCLE)

        # Stop the motor, for security purpose
        self.stop()

    def stop(self):
        """
        Function that stops a motor.
        """
        # Put the PWM duty cycle to 0
        self.speed_pwm.ChangeDutyCycle(0)

        # Put all the digital pins to LOW
        GPIO.output(self.forward_pin, GPIO.LOW)
        GPIO.output(self.backward_pin, GPIO.LOW)

    def run(self, signed_speed_percent):
        """
        Function that moves a motor with a given speed percent.
        :param signed_speed_percent: motion speed, between -100 and +100.
        A positive speed moves wheels forwards, negative moves backwards.
        """
        # Change motor speed, modifying the duty cycle (assuring that it's between 0 and 100)
        duty_cycle = clip(abs(signed_speed_percent), 0, 100)
        self.speed_pwm.ChangeDutyCycle(duty_cycle)

        # Set the output of the digital pins accordingly to the direction
        if signed_speed_percent >= 0:
            GPIO.output(self.forward_pin, GPIO.HIGH)
            GPIO.output(self.backward_pin, GPIO.LOW)
        else:
            GPIO.output(self.forward_pin, GPIO.LOW)
            GPIO.output(self.backward_pin, GPIO.HIGH)


class MotionDriver(Node):
    """
    ROS2 node that controls AlphaBot2's motion (Toshiba TB6612FNG dual motor driver).
    It is subscribed to the "cmd_vel" (Twist msg) topic and to the "obstacles" (Obstacle msg) topic.
    """
    def __init__(self,
                 l_motor_fw_pin=AIN2_PIN,
                 l_motor_bw_pin=AIN1_PIN,
                 l_motor_speed_pwm_pin=PWMA_PIN,
                 r_motor_fw_pin=BIN2_PIN,
                 r_motor_bw_pin=BIN1_PIN,
                 r_motor_speed_pwm_pin=PWMB_PIN):

        super().__init__("motion_driver")

        self.get_logger().info("Node init ...")

        # Create the timer (called by rclpy.spin()) with its callback function
        self.spin_timer = self.create_timer(SPIN_TIMER_PERIOD_SEC, self.spin_timer_callback)

        # Topics subscriptions
        self.cmd_vel_sub = self.create_subscription(Twist, CMD_VEL_TOPIC, self.cmd_vel_sub_callback, 10)
        self.obstacles_sub = self.create_subscription(Obstacle, OBSTACLES_TOPIC, self.obstacles_sub_callback, 10)

        # L and R motor instances
        self.left_motor = Motor(l_motor_fw_pin, l_motor_bw_pin, l_motor_speed_pwm_pin)
        self.right_motor = Motor(r_motor_fw_pin, r_motor_bw_pin, r_motor_speed_pwm_pin)

        # Internal status
        self.left_signed_speed_percent = 0   # between -100 and +100
        self.right_signed_speed_percent = 0  # between -100 and +100

        self.last_received_vel_timestamp_sec = self.get_clock().now().to_msg().sec  # timestamp of the last
                                                                                    # received msg on "cmd_vel"
        self.obstacle_detected = False  # True if there is an obstacle
        self.braked = False             # True if the robot has been braked

        self.get_logger().info("Node init complete.")

    def brake(self):
        """
        Function that brakes the motors.
        To improve braking performance, it make the motors move with opposite speed for some ms in order not to let them
        free to inertia, then it stops the motors.
        """
        self.braked = True

        # Move the motors with opposite speed for some ms
        self.left_motor.run(-self.left_signed_speed_percent)
        self.right_motor.run(-self.right_signed_speed_percent)

        # Braking duration is proportional to a fixed maximum value
        max_abs_speed_percent = max(abs(self.right_signed_speed_percent), abs(self.left_signed_speed_percent))
        time.sleep(MAX_BRAKING_DURATION_SEC * max_abs_speed_percent / 100)

        # Stop the motors
        self.left_motor.stop()
        self.right_motor.stop()

    def move(self):
        """
        Function that moves the motors according to the current left/right signed speed percent.
        """
        self.braked = False

        self.left_motor.run(self.left_signed_speed_percent)
        self.right_motor.run(self.right_signed_speed_percent)

    def set_wheels_speed(self, linear_velocity, angular_rate):
        """
        Function that sets speed_percent for both left and right motor, starting from linear_velocity and angular_rate.
        If the desired speed exceeds motor limits, the speed is normalized according to the motor's capability.
        :param linear_velocity: desired linear velocity, in m/s
        :param angular_rate: desired angular rate, in rad/s
        """

        """
        Ideally we'd use the desired wheel speeds along with data from wheel speed sensor to come up with the power
        we need to apply to the motors, but AlphaBot2 does not have wheel speed sensors.
        Instead, we have to convert (linear_velocity, angular_rate) in RPM and then from RPM to percent of max wheel
        rpm. We'll use this percent as duty cycle to apply to each motor.
        
        Convert the (linear_velocity, angular_rate) in RPM, both for left and right wheel:
            left_rpm = (linear_velocity - 0.5 * angular_rate * WHEEL_DIST) / (RPM_TO_RAD_PER_S * DIST_PER_RAD)
            right_rpm = (linear_velocity + 0.5 * angular_rate * WHEEL_DIST) / (RPM_TO_RAD_PER_S * DIST_PER_RAD)
        where:
            RPM_TO_RAD_PER_S = 2 * pi / 60
            DIST_PER_RAD = (2 * pi * WHEEL_RADIUS) / (2 * pi) = WHEEL_RADIUS
        
        Convert from RPM to speed_percent:
            wheel_speed_percent = 100 * wheel_rpm / MAX_WHEEL_RPM
        
        REF: https://robotics.stackexchange.com/questions/18048/inverse-kinematics-for-differential-robot-knowing-linear-and-angular-velocities
        
        Robot maximum theoretical capabilities:
            Li-ion 3.7V full charged:
                MAX_LINEAR_VELOCITY = (MAX_WHEEL_RPM / 60) * 2 * pi * WHEEL_RADIUS = 1.65 m/s
                MAX_ANGULAR_RATE = (MAX_LINEAR_VELOCITY / ROBOT_BASE_CIRC) * 2 * pi =
                                 = MAX_LINEAR_VELOCITY / (pi * WHEEL_DIST) * 2 * pi = 38.8 rad/s
            USB powered:
                MAX_LINEAR_VELOCITY = 0.88 m/s
                MAX_ANGULAR_RATE = 20.71 rad/s
        """

        # Convert from (linear_velocity, angular_rate) to RPM
        left_rpm = (linear_velocity - 0.5 * angular_rate * WHEELS_DIST_M) / (RPM_TO_RAD_PER_SEC * WHEEL_RADIUS_M)
        right_rpm = (linear_velocity + 0.5 * angular_rate * WHEELS_DIST_M) / (RPM_TO_RAD_PER_SEC * WHEEL_RADIUS_M)

        # Normalize RPMs (with respect to the max value) if they exceed motor's capability
        max_abs_rpm = max(abs(left_rpm), abs(right_rpm))
        if max_abs_rpm > MAX_MOTOR_RPM:
            exceed_ratio = max_abs_rpm / MAX_MOTOR_RPM
            left_rpm = left_rpm / exceed_ratio
            right_rpm = right_rpm / exceed_ratio

        # Convert from RPM to speed_percent
        self.left_signed_speed_percent = (100 * left_rpm / MAX_MOTOR_RPM)  # between -100 and +100
        self.right_signed_speed_percent = (100 * right_rpm / MAX_MOTOR_RPM)

    def spin_timer_callback(self):
        """
        Function that moves the motors according to the messages on the "cmd_vel" topic.
        It brakes the motors if no messages have been received on "cmd_vel" for a certain time.
        The function is called-back by the Timer, "spinned" by rclpy.
        """
        # Compute the time interval between now and the timestamp of the last received msg on "cmd_vel"
        delay_sec = self.get_clock().now().to_msg().sec - self.last_received_vel_timestamp_sec

        if delay_sec < MOTOR_BRAKE_TIMEOUT_SEC:
            self.get_logger().info(f"Moving >> left_signed_speed_percent={self.left_signed_speed_percent:.2f}, "
                                   f"right_signed_speed_percent={self.right_signed_speed_percent:.2f}")
            self.move()
        else:
            # If we haven't received new commands for a while, we may have lost contact with the vel commander.
            # We stop the robot.
            if not self.braked:
                self.get_logger().warn(f"Braking >> no cmd_vel received for >{MOTOR_BRAKE_TIMEOUT_SEC}s")
                self.brake()

    def cmd_vel_sub_callback(self, cmd_vel_msg):
        """
        Function called when there is a new Twist message on "cmd_vel" topic.
        It sets the speeds (-100 < x < 100) that have to be applied to the wheels, considering eventual obstacles.
        :param cmd_vel_msg: received Twist message.
        """
        # Update the timestamp
        self.last_received_vel_timestamp_sec = self.get_clock().now().to_msg().sec

        # Get the linear and the angular components
        linear = cmd_vel_msg.linear.x
        angular = cmd_vel_msg.angular.z
        self.get_logger().info(f"Received cmd_vel >> "
                               f"linear={cmd_vel_msg.linear.x:.3}, angular={cmd_vel_msg.angular.z:.3}")

        # Prevent forward motion if there is an obstacle
        if self.obstacle_detected and linear > 0:
            self.get_logger().warn(f"Forced linear=0 >> forward motion not permitted due to an obstacle")
            linear = 0

        # Set wheels speed
        self.set_wheels_speed(linear_velocity=linear, angular_rate=angular)

    def obstacles_sub_callback(self, obstacle_msg):
        """
        Function called when there is a new Obstacle message on "obstacles" topic.
        It brakes the motors if there are obstacles.
        :param obstacle_msg: received Obstacle message.
        """
        # If there is an obstacle
        if obstacle_msg.left_obstacle or obstacle_msg.right_obstacle:
            # If the motors have not been braked and the obstacle has not been detected yet
            if not self.braked and not self.obstacle_detected:
                self.obstacle_detected = True
                self.get_logger().warn(f"Braking >> obstacle detected")
                self.brake()
        else:
            self.obstacle_detected = False


def main(args=None):
    """
    Main function of the motion_driver node.
    """
    rclpy.init(args=args)
    gpio_init()

    # Create, spin and destroy the node
    motion_driver = MotionDriver()
    rclpy.spin(motion_driver)
    motion_driver.destroy_node()

    gpio_close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
