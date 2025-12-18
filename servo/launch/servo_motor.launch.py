from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    can_interface = LaunchConfiguration('can_interface')
    control_hz = LaunchConfiguration('control_hz')
    auto_start = LaunchConfiguration('auto_start')

    return LaunchDescription([
        # Global arguments
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='SocketCAN interface'
        ),
        DeclareLaunchArgument(
            'control_hz',
            default_value='50.0',
            description='Control loop frequency'
        ),
        DeclareLaunchArgument(
            'auto_start',
            default_value='true',
            description='Automatically power on motors'
        ),

        # -------- Motor 1 --------
        Node(
            package='servo',
            executable='servo_motor_node',
            name='servo_motor_1',
            output='screen',
            parameters=[{
                'can_interface': can_interface,
                'can_id': 1,
                'joint_name': 'motor1',
                'control_hz': control_hz,
                'auto_start': auto_start,
            }],
        ),

        # -------- Motor 2 --------
        Node(
            package='servo',
            executable='servo_motor_node',
            name='servo_motor_2',
            output='screen',
            parameters=[{
                'can_interface': can_interface,
                'can_id': 2,
                'joint_name': 'motor2',
                'control_hz': control_hz,
                'auto_start': auto_start,
                'reverse_polarity': True,
            }],
        ),

        # -------- Motor 3 --------
        Node(
            package='servo',
            executable='servo_motor_node',
            name='servo_motor_3',
            output='screen',
            parameters=[{
                'can_interface': can_interface,
                'can_id': 3,
                'joint_name': 'motor3',
                'control_hz': control_hz,
                'auto_start': auto_start, 
            }],
        ),

        # -------- Motor 4 --------
        Node(
            package='servo',
            executable='servo_motor_node',
            name='servo_motor_4',
            output='screen',
            parameters=[{
                'can_interface': can_interface,
                'can_id': 4,
                'joint_name': 'motor4',
                'control_hz': control_hz,
                'auto_start': auto_start,
                'reverse_polarity': True,
            }],
        ),
    ])

