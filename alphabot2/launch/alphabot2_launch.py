from launch import LaunchDescription
from launch_ros.actions import Node


NAMESPACE = "alphabot2"
IMAGE_SIZE = "[320,240]"

MOTION_DRIVER_LOG_LVL = "WARN"
IR_OBSTACLE_SENSORS_LOG_LVL = "WARN"
VIRTUAL_ODOMETER_LOG_LVL = "WARN"
QR_DETECTOR_LOG_LVL = "WARN"
V4L2_CAMERA_LOG_LVL = "FATAL"


def generate_launch_description():
    launch_description = LaunchDescription()

    motion_driver_node = Node(
        package="alphabot2",
        namespace=NAMESPACE,
        executable="motion_driver",
        output="screen",
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', MOTION_DRIVER_LOG_LVL],
    )

    ir_obstacle_sensors_node = Node(
        package="alphabot2",
        namespace=NAMESPACE,
        executable="IR_obstacle_sensors",
        output="screen",
        emulate_tty=True,
        arguments=['--ros-args',
                   '--log-level', IR_OBSTACLE_SENSORS_LOG_LVL],
    )

    virtual_odometer_node = Node(
        package="alphabot2",
        namespace=NAMESPACE,
        executable="virtual_odometer",
        output="screen",
        emulate_tty=True,
        arguments=['--ros-args',
                   '--log-level', VIRTUAL_ODOMETER_LOG_LVL],
    )

    qr_detector_node = Node(
        package="alphabot2",
        namespace=NAMESPACE,
        executable="QR_detector",
        output="screen",
        emulate_tty=True,
        arguments=['--ros-args',
                   '--log-level', QR_DETECTOR_LOG_LVL],
    )

    v4l2_camera_node = Node(
        package="v4l2_camera",
        namespace=NAMESPACE,
        executable="v4l2_camera_node",
        output="screen",
        emulate_tty=True,
        arguments=['--ros-args',
                   '--log-level', V4L2_CAMERA_LOG_LVL,
                   '-p', f'image_size:={IMAGE_SIZE}'],
    )

    launch_description.add_action(ir_obstacle_sensors_node)
    launch_description.add_action(motion_driver_node)
    launch_description.add_action(virtual_odometer_node)
    launch_description.add_action(v4l2_camera_node)
    launch_description.add_action(qr_detector_node)

    return launch_description
