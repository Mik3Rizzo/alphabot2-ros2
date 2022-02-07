import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from alphabot2_interfaces.msg import Obstacle

VIRTUAL_ODOMETRY_TOPIC = "virtual_odometry"
CMD_VEL_TOPIC = "cmd_vel"
OBSTACLES_TOPIC = "obstacles"
SPIN_TIMER_PERIOD_SEC = 0.025   # Timer callback period (40 Hz)
CMD_VEL_PUB_PERIOD_SEC = 0.025  # cmd_vel publishing period (40 Hz)


class VirtualOdometer(Node):
    """
    ROS2 node that emulates a virtual odometer since AlphaBot2 doesn't have any odometry oriented sensor.
    It is subscribed to the "cmd_vel" (Twist msg) and "obstacles" (Obstacle msg) topics and it publishes a speed
    estimation (Twist msg) on the "virtual_odometry" topic.
    """
    def __init__(self):

        super().__init__("virtual_odometer")

        self.get_logger().info("Node init ...")

        # Create the timer (called by rclpy.spin()) with its callback function
        self.timer = self.create_timer(SPIN_TIMER_PERIOD_SEC, self.virtual_odometry_pub_callback)

        # Topics publisher and subscribers
        self.cmd_vel_sub = self.create_subscription(Twist, CMD_VEL_TOPIC, self.cmd_vel_sub_callback, 10)
        self.obstacles_sub = self.create_subscription(Obstacle, OBSTACLES_TOPIC, self.obstacles_sub_callback, 10)
        self.virtual_odometry_pub = self.create_publisher(Twist, VIRTUAL_ODOMETRY_TOPIC, 10)

        # Internal status
        self.linear = 0                 # linear velocity
        self.angular = 0                # angular rate
        self.obstacle_detected = False  # True if there is an obstacle

        self.get_logger().info("Node init complete.")

    def cmd_vel_sub_callback(self, twist_msg):
        """
        Function called when there is a new Twist message on the "cmd_vel" topic.
        :param twist_msg: received Twist message.
        """
        # Set linear and angular copying them from the "cmd_vel" message
        self.linear = twist_msg.linear.x
        self.angular = twist_msg.angular.z

    def obstacles_sub_callback(self, obstacle_msg):
        """
        Function called when there is a new Obstacle message on the "obstacles" topic.
        :param obstacle_msg: received Obstacle message.
        """
        # Update the internal status
        self.obstacle_detected = obstacle_msg.right_obstacle or obstacle_msg.left_obstacle

    def virtual_odometry_pub_callback(self):
        """
        Function called to publish a new Twist message on "virtual_odometry" topic.
        This represents the actual estimated speed of the robot.
        """
        # Create a Twist message
        twist_msg = Twist()
        twist_msg.linear.x = float(self.linear)
        twist_msg.angular.z = float(self.angular)

        # Prevent forward motion if there is an obstacle as it's done in the motion_driver
        if self.obstacle_detected and twist_msg.linear.x > 0:
            twist_msg.linear.x = float(0)

        # Publish the message
        self.virtual_odometry_pub.publish(twist_msg)
        self.get_logger().info(f"Publishing [{VIRTUAL_ODOMETRY_TOPIC}] >> "
                               f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}")


def main(args=None):
    """
    Main function of the virtual_odometer node.
    """
    rclpy.init(args=args)

    # Create, spin and destroy the node
    virtual_odometer = VirtualOdometer()
    rclpy.spin(virtual_odometer)
    virtual_odometer.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
