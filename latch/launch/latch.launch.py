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
                        "pwm_chip": 0,
                        "pwm_channel": 0,
                        "period_us": 20000,
                        "min_pulse_us": 1000,
                        "max_pulse_us": 2000,
                        "min_angle_deg": 0.0,
                        "max_angle_deg": 180.0,
                        "neutral_angle_deg": 90.0,
                        "auto_enable": True,
                        "startup_angle_deg": 90.0,
                    }
                ],
            )
        ]
    )
