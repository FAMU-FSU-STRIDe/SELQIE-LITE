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
                        "gpio_pin": 15,
                        "gpio_mode": "BOARD",
                        "active_high": True,
                    }
                ],
            )
        ]
    )
