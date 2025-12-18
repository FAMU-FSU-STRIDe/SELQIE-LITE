from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Bring up SELQIE hardware: four motors using servo control."""

    params_common = {
        'can_interface': 'can0',
        'motor_type': 'AK40-10',
        'control_hz': 50.0,
        'auto_start': True,
        'reverse_polarity': False,
    }

    return LaunchDescription([
        # --- Four Motor Nodes ---
        Node(
            package='servo',
            executable='servo_motor_node',
            name='servo_motor1',
            parameters=[{**params_common, 'can_id': 1, 'joint_name': 'motor1', 'reverse_polarity': True}],
        ),
        Node(
            package='servo',
            executable='servo_motor_node',
            name='servo_motor2',
            parameters=[{**params_common, 'can_id': 2, 'joint_name': 'motor2'}],
        ),
        Node(
            package='servo',
            executable='servo_motor_node',
            name='servo_motor3',
            parameters=[{**params_common, 'can_id': 3, 'joint_name': 'motor3', 'reverse_polarity': True}],
        ),
        Node(
            package='servo',
            executable='servo_motor_node',
            name='servo_motor4',
            parameters=[{**params_common, 'can_id': 4, 'joint_name': 'motor4'}],
        ),
    ])
