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
                        "port": "/dev/ttyTHS1",
                        "baud": 115200,
                        "rate_hz": 2.0,
                        "timeout_s": 0.25,
                        "wakeup_send_twice": True,
                    }
                ],
            )
        ]
    )
