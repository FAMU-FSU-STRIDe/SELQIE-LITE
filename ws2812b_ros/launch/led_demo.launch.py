# launch/led_demo.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # LED driver params
    num_leds     = LaunchConfiguration('num_leds')
    brightness   = LaunchConfiguration('brightness')
    spi_bus      = LaunchConfiguration('spi_bus')
    spi_dev      = LaunchConfiguration('spi_dev')
    spi_hz       = LaunchConfiguration('spi_hz')
    pixel_order  = LaunchConfiguration('pixel_order')

    # Tester params
    rate_hz      = LaunchConfiguration('rate_hz')
    hold_secs    = LaunchConfiguration('hold_secs')

    return LaunchDescription([
        # Driver args
        DeclareLaunchArgument('num_leds',    default_value='2'),
        DeclareLaunchArgument('brightness',  default_value='0.2'),
        DeclareLaunchArgument('spi_bus',     default_value='0'),
        DeclareLaunchArgument('spi_dev',     default_value='0'),
        DeclareLaunchArgument('spi_hz',      default_value='2400000'),
        DeclareLaunchArgument('pixel_order', default_value='GRB'),
        # Tester args
        DeclareLaunchArgument('rate_hz',     default_value='5.0'),   # publish rate
        DeclareLaunchArgument('hold_secs',   default_value='1.0'),   # how long each color holds

        Node(
            package='ws2812b_ros',
            executable='led_node',
            name='ws2812b_led_node',
            output='screen',
            parameters=[{
                'num_leds':    num_leds,
                'brightness':  brightness,
                'spi_bus':     spi_bus,
                'spi_dev':     spi_dev,
                'spi_hz':      spi_hz,
                'pixel_order': pixel_order,
            }],
        ),

        Node(
            package='ws2812b_ros',
            executable='led_tester',
            name='ws2812b_led_tester',
            output='screen',
            parameters=[{
                'num_leds':   num_leds,
                'rate_hz':    rate_hz,
                'hold_secs':  hold_secs,
            }],
        ),
    ])

