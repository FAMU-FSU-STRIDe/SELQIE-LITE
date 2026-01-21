from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ping1d_node = Node(
        package='ms5837_bar_ros',
        executable='bar30_node',
        output="screen",
    )

    nodes = [
        ping1d_node,
    ]

    return LaunchDescription(nodes)
