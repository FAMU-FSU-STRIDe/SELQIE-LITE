from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ping1d_node = Node(
        package='ms5837_bar_ros',
        executable='bar30_node',
        output="screen",
    )

    base_to_range = Node(
            ## Configure the TF of the robot to the origin of the map coordinates
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0', '0.0', '0.0', 'base_link', 'bar30_link']
    )

    nodes = [
        ping1d_node,
        base_to_range,
    ]

    return LaunchDescription(nodes)
