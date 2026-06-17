import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

MOTOR_LAUNCH_FILE = os.path.join(
        get_package_share_directory('servo'), 'launch', 'servo_motor.launch.py')

LEAK_SENSOR_LAUNCH_FILE = os.path.join(
        get_package_share_directory('leak_sensor'), 'launch', 'leak_sensor.launch.py')
        
LED_LAUNCH_FILE = os.path.join(
        get_package_share_directory('ws2812b_ros'), 'launch', 'leds.launch.py')
        
LATCH_LAUNCH_FILE = os.path.join(
        get_package_share_directory('latch'), 'launch', 'latch.launch.py')

BATTERY_LAUNCH_FILE = os.path.join(
        get_package_share_directory('battery'), 'launch', 'tinybms_voltage_uart.launch.py')
        
DEPTH_LAUNCH_FILE = os.path.join(
        get_package_share_directory('ms5837_bar_ros'), 'launch', 'bar30.launch.py')
        
IMU_LAUNCH_FILE = os.path.join(
        get_package_share_directory('bno08x_driver'), 'launch', 'bno085_i2c.launch.py')

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(MOTOR_LAUNCH_FILE)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(LATCH_LAUNCH_FILE)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(LEAK_SENSOR_LAUNCH_FILE)
        ),
#        IncludeLaunchDescription(
#            PythonLaunchDescriptionSource(BATTERY_LAUNCH_FILE)
#        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(LED_LAUNCH_FILE)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(DEPTH_LAUNCH_FILE)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(IMU_LAUNCH_FILE)
        ),
    ])
