from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="battery",
                executable="tinybms_voltage_uart",
                name="tinybms_voltage_uart",
                output="screen",
                parameters=[
                    {
                        "port": "/dev/ttyUSB0",
                        "rate_hz": 5.0,
                    }
                ],
            )
        ]
    )
