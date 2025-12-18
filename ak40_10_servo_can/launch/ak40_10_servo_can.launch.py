from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch args
    can_interface = LaunchConfiguration("can_interface")
    motor_id = LaunchConfiguration("motor_id")
    joint_name = LaunchConfiguration("joint_name")
    mode = LaunchConfiguration("mode")
    control_hz = LaunchConfiguration("control_hz")
    reverse_polarity = LaunchConfiguration("reverse_polarity")
    pole_pairs = LaunchConfiguration("pole_pairs")
    command_timeout_s = LaunchConfiguration("command_timeout_s")

    return LaunchDescription([
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument("motor_id", default_value="1"),
        DeclareLaunchArgument("joint_name", default_value="ak40_10"),
        # pos | pos_spd | rpm | current | duty | brake_current
        DeclareLaunchArgument("mode", default_value="pos_spd"),
        DeclareLaunchArgument("control_hz", default_value="200.0"),
        DeclareLaunchArgument("reverse_polarity", default_value="false"),
        # optional: if you set this, JointState velocity becomes meaningful
        DeclareLaunchArgument("pole_pairs", default_value="0"),
        DeclareLaunchArgument("command_timeout_s", default_value="0.25"),

        Node(
            package="ak40_10_servo_can",
            executable="ak40_node",   # <-- change if your console_script name differs
            name="ak40_10_servo_can",
            output="screen",
            parameters=[{
                "can_interface": can_interface,
                "motor_id": motor_id,
                "joint_name": joint_name,
                "mode": mode,
                "control_hz": control_hz,
                "reverse_polarity": reverse_polarity,
                "pole_pairs": pole_pairs,
                "command_timeout_s": command_timeout_s,
            }],
        ),
    ])

