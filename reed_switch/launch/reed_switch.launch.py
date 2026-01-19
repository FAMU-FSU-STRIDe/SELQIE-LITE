from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="reed_switch",
            executable="reed_switch_node",
            name="reed_switch",
            output="screen",
            parameters=[
                {
                    "gpio_pin": 29,
                    "gpio_mode": "BOARD",
                    "pull": "UP",
                    "active_high": True,
                    "poll_hz": 10.0,
                }
            ],
        )
    ])
