from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="leak_sensor",
            executable="leak_sensor_node",
            name="leak_sensor",
            output="screen",
            parameters=[
                {
                    "gpio_pin": 33,
                    "gpio_mode": "BOARD",
                    "pull": "DOWN",
                    "active_high": True,
                    "poll_hz": 10.0,
                }
            ],
        )
    ])
