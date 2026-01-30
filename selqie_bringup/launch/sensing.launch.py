import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

BAR30_LAUNCH_FILE = os.path.join(
        get_package_share_directory('sensing_bringup'), 'launch', 'bar30.launch.py')

IMU_LAUNCH_FILE = os.path.join(
        get_package_share_directory('sensing_bringup'), 'launch', 'imu.launch.py')

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(BAR30_LAUNCH_FILE)
        ),
#        IncludeLaunchDescription(
#            PythonLaunchDescriptionSource(IMU_LAUNCH_FILE)
#        ),
    ])
