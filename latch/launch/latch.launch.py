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
                        "port": "/dev/ttyACM0",
                        "baud": 115200,
                        "timeout_s": 0.2,
                    }
                ],
            )
        ]
    )
