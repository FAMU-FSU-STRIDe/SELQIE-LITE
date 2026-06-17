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
                    "gpio_pin": 16,
                    "gpio_mode": "BOARD",
                    "pull": "UP",
                    "active_high": False,
                    "poll_hz": 10.0,
                    "servo_pin": 32,
                    "servo_hz": 50.0,
                    "servo_idle_duty": 5.0,
                    "servo_active_duty": 10.0,
                }
            ],
        )
    ])
