from setuptools import setup, find_packages

package_name = 'ws2812b_ros'

setup(
    name=package_name,
    version='0.0.4',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/led_demo.launch.py']),
        ('share/' + package_name + '/launch', [
                'launch/led_demo.launch.py',
                'launch/rainbow_fade.launch.py',   # <-- add this
            ]),
    ],
    install_requires=[
        'setuptools',
        'spidev',
        'rclpy',
        'std_msgs',
    ],
    zip_safe=True,
    maintainer='Ryan Kaczmarczyk',
    maintainer_email='',
    description='WS2812B (NeoPixel) SPI driver node for ROS 2 with CLI and demo',
    license='MIT',
    entry_points={
        'console_scripts': [
            'led_node = ws2812b_ros.led_node:main',
            'led_tester = ws2812b_ros.led_tester:main',
            'ws2812b-set = ws2812b_ros.cli:main',
            'rainbow_fade = ws2812b_ros.rainbow_fade:main',   # <-- add this
        ],
    },
)

