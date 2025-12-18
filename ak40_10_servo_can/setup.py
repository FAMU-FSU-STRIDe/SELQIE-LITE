from glob import glob
import os
from setuptools import setup

package_name = 'ak40_10_servo_can'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'python-can'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'ak40_node = ak40_10_servo_can.ak40_node:main',
        ],
    },
)

