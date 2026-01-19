from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ws2812b_ros',
            executable='led_node',
            name='ws2812b_led_node',
            parameters=[
                {'num_leds': 2},
            ],
        ),
        Node(
            package='ws2812b_ros',
            executable='ws2812b-set',
            name='ws2812b_led_default',
            arguments=['--hex', '0000FF', '--num-leds', '2'],
        ),
    ])
