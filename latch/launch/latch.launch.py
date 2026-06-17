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
                        "servo_pin": 32,
                        "gpio_mode": "BOARD",
                    }
                ],
            )
        ]
    )
