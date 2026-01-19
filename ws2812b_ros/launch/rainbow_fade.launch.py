# launch/rainbow_fade.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Driver (hardware) params
    num_leds    = LaunchConfiguration('num_leds')
    brightness  = LaunchConfiguration('brightness')
    spi_bus     = LaunchConfiguration('spi_bus')
    spi_dev     = LaunchConfiguration('spi_dev')
    spi_hz      = LaunchConfiguration('spi_hz')
    pixel_order = LaunchConfiguration('pixel_order')

    # Animation (publisher) params
    rate_hz        = LaunchConfiguration('rate_hz')
    cycle_seconds  = LaunchConfiguration('cycle_seconds')
    spread_degrees = LaunchConfiguration('spread_degrees')
    saturation     = LaunchConfiguration('saturation')
    value          = LaunchConfiguration('value')
    reverse        = LaunchConfiguration('reverse')

    return LaunchDescription([
        # Driver args
        DeclareLaunchArgument('num_leds',    default_value='2'),
        DeclareLaunchArgument('brightness',  default_value='0.2'),
        DeclareLaunchArgument('spi_bus',     default_value='0'),
        DeclareLaunchArgument('spi_dev',     default_value='0'),
        DeclareLaunchArgument('spi_hz',      default_value='2400000'),
        DeclareLaunchArgument('pixel_order', default_value='GRB'),

        # Animation args
        DeclareLaunchArgument('rate_hz',        default_value='60.0'),
        DeclareLaunchArgument('cycle_seconds',  default_value='8.0'),
        DeclareLaunchArgument('spread_degrees', default_value='180.0'),
        DeclareLaunchArgument('saturation',     default_value='1.0'),
        DeclareLaunchArgument('value',          default_value='1.0'),
        DeclareLaunchArgument('reverse',        default_value='false'),

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
            executable='rainbow_fade',
            name='ws2812b_rainbow_fade',
            output='screen',
            parameters=[{
                'num_leds':        num_leds,
                'rate_hz':         rate_hz,
                'cycle_seconds':   cycle_seconds,
                'spread_degrees':  spread_degrees,
                'saturation':      saturation,
                'value':           value,
                'reverse':         reverse,
            }],
        ),
    ])

