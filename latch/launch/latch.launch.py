from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="latch",
                executable="latch_node",
                name="latch_node",
                output="screen",
                parameters=[
                    {
                        "port": "/dev/serial/by-id/usb-Teensyduino_USB_Serial_14238250-if00",
                        "baud": 115200,
                        "timeout_s": 0.2,
                    }
                ],
            )
        ]
    )
