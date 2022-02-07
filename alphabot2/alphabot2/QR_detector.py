import numpy as np
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

QR_CODES_TOPIC = "qr_codes"
COMPRESSED_IMAGE_TOPIC = "image_raw/compressed"
SPIN_TIMER_PERIOD_SEC = 0.5     # Timer callback period (2 Hz)

CAMERA_IMAGE_HEIGHT_PX = 240    # height (in pixels) of the images coming from the camera
QR_MIN_HEIGHT_PX = 170          # the QR height has to be at least 170 px. Experimentally, with this constraint the QR
                                # is detected if it is distant between 17 and 22 cm from the camera


class QRDetector(Node):
    """
    ROS2 node that analyzes if there is a QR captured by the camera and publishes the QR content on the "qr_codes" topic
    as a String (eventually empty if no code is detected).
    It is subscribed to the "image_raw/compressed" (CompressedImage msg) topic.
    """
    def __init__(self):

        super().__init__("QR_detector")

        self.get_logger().info("Node init ...")

        # Create the timer (called by rclpy.spin()) with its callback function
        self.timer = self.create_timer(SPIN_TIMER_PERIOD_SEC, self.qr_codes_pub_callback)

        # Topics publisher and subscriber
        self.compressed_image_sub = self.create_subscription(CompressedImage, COMPRESSED_IMAGE_TOPIC,
                                                             self.compressed_image_sub_callback, 10)
        self.qr_codes_pub = self.create_publisher(String, QR_CODES_TOPIC, 10)

        # CVBridge and OpenCV's QRCodeDetector objects
        self.cv_bridge = CvBridge()
        self.qr_detector = cv2.QRCodeDetector()

        # Internal status
        self.last_compressed_image = CompressedImage()  # last CompressedImage received from the camera
        self.qr_content = ""                            # content of the last detected QR

        self.get_logger().info("Node init complete.")

    def compressed_image_sub_callback(self, compressed_image_msg):
        """
        Function called when there is a new CompressedImage message on the "image_raw/compressed" topic.
        :param compressed_image_msg: received CompressedImage message.
        """
        # Update the internal status
        self.last_compressed_image = compressed_image_msg

    def qr_codes_pub_callback(self):
        """
        Function called to retrieve the QR content from the last received image and publish it as a String message on
        the "qr_codes" topic.
        If no QR code is detected, it will publish and empty string.
        """
        # Decode the QR from the last received image (if it is present)
        self.decode_qr()

        # Create a String message
        string_msg = String()
        string_msg.data = self.qr_content

        # Publish the message
        self.qr_codes_pub.publish(string_msg)
        self.get_logger().info(f"Publishing [{QR_CODES_TOPIC}] >> qr={string_msg.data}")

    def decode_qr(self):
        """
        Function that detects if there is a QR in the last received image and gets the QR string content using OpenCV.
        If no QR code is detected or if the QR is too far from the robot, the content will be an empty string.
        """
        # Reset QR content
        self.qr_content = ""

        # Convert from CompressedImage to a OpenCV image using cv_bridge
        cv2_image = self.cv_bridge.compressed_imgmsg_to_cv2(self.last_compressed_image, desired_encoding="passthrough")

        # Detect the QR and decode the related string (first element of the tuple)
        qr_content, qr_points, _ = self.qr_detector.detectAndDecode(cv2_image)

        if qr_content:
            # Actually qr_points is a list of list: [[top_left, top_right, bottom_right, bottom_left]]
            # we are interested in the inner part
            qr_vertices = qr_points[0]

            # Calculate the size of the QR edge computing the Euclidean distance (norm) between the top_left and the
            # bottom_left vertices
            qr_height_px = np.linalg.norm(qr_vertices[0] - qr_vertices[3])

            self.get_logger().info(f"Debugging [{QR_CODES_TOPIC}] >> "
                                   f"qr_vertices={qr_vertices}, "
                                   f"qr_height_px={qr_height_px:.2}")

            # Consider the QR only if it is big enough (i.e. if the robot is near it)
            if qr_height_px > QR_MIN_HEIGHT_PX:
                self.qr_content = qr_content


def main(args=None):
    """
    Main function of the QR_detector node.
    """
    np.set_printoptions(suppress=True)  # Prevent numpy scientific notation
    rclpy.init(args=args)

    qr_detector = QRDetector()
    rclpy.spin(qr_detector)
    qr_detector.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
