from setuptools import setup
from glob import glob
import os

package_name = 'mjbots_power_dist'

launch_files = glob('launch/*.py')

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', launch_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SELQIE Maintainers',
    maintainer_email='maintainers@example.com',
    description='Read MJBots power distribution telemetry over SocketCAN and republish it to ROS 2.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'power_dist_node = mjbots_power_dist.power_dist_node:main',
        ],
    },
)
