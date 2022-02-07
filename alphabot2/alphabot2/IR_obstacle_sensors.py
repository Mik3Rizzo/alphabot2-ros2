import rclpy
from rclpy.node import Node

from alphabot2_interfaces.msg import Obstacle

import RPi.GPIO as GPIO

"""
AlphaBot2 is equipped with a couple of ST188 reflective infrared photoelectric sensors, for obstacles collision 
detection.
Each sensor has one output that is LOW if there is an obstacle, HIGH otherwise (inverse logic).
"""

# ST188 / Raspberry Pi GPIO pin mapping
DL_PIN = 16  # L-sensor output pin
DR_PIN = 19  # R-sensor output pin

OBSTACLES_TOPIC = "obstacles"
SPIN_TIMER_PERIOD_SEC = 0.025  # Timer callback period (40 Hz)


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


class IRObstacleSensors(Node):
    """
    ROS2 node that controls AlphaBot2's IR obstacle sensors (ST188 reflective infrared photoelectric sensors).
    It publishes Obstacle messages on the "obstacles" topic.
    """
    def __init__(self, left_sensor_pin=DL_PIN, right_sensor_pin=DR_PIN):

        super().__init__("IR_obstacle_sensors")

        self.get_logger().info("Node init ...")

        # Setup pin numbers
        self.left_sensor_pin = left_sensor_pin
        self.right_sensor_pin = right_sensor_pin

        # Setup pins as INPUT
        GPIO.setup(self.left_sensor_pin, GPIO.IN, GPIO.PUD_UP)
        GPIO.setup(self.right_sensor_pin, GPIO.IN, GPIO.PUD_UP)

        # Create the timer (called by rclpy.spin()) with its callback function
        self.timer = self.create_timer(SPIN_TIMER_PERIOD_SEC, self.obstacles_pub_callback)

        # Topic publisher
        self.obstacles_pub = self.create_publisher(Obstacle, OBSTACLES_TOPIC, 10)

        # Internal status (True if there is an obstacle on the left/right)
        self.left_obstacle = not GPIO.input(self.left_sensor_pin)  # the sensor has an inverse logic
        self.right_obstacle = not GPIO.input(self.right_sensor_pin)

        self.get_logger().info("Node init complete.")

    def check_for_obstacles(self):
        """
        Function that checks if the two sensors are signaling an obstacle and updates the internal status.
        Please note that the specific sensors have an inverse logic.
        """
        self.left_obstacle = not GPIO.input(self.left_sensor_pin)
        self.right_obstacle = not GPIO.input(self.right_sensor_pin)

    def obstacles_pub_callback(self):
        """
        Function called to publish a new Obstacle message on "obstacles" topic.
        """
        self.check_for_obstacles()

        # Create the Obstacle message
        obstacle_msg = Obstacle()
        obstacle_msg.left_obstacle = self.left_obstacle
        obstacle_msg.right_obstacle = self.right_obstacle

        # Publish the message
        self.obstacles_pub.publish(obstacle_msg)
        self.get_logger().info(f"Publishing >> left_obstacle={self.left_obstacle}, right_obstacle={self.right_obstacle}")


def main(args=None):
    """
    Main function of the IR_obstacle_sensors node.
    """
    rclpy.init(args=args)
    gpio_init()

    # Create, spin and destroy the node
    ir_obstacle_sensors = IRObstacleSensors()
    rclpy.spin(ir_obstacle_sensors)
    ir_obstacle_sensors.destroy_node()

    gpio_close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
