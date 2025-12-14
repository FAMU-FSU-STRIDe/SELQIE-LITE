from setuptools import setup
from glob import glob
import os

package_name = 'quad_legs'

# Collect any launch files that actually exist
launch_files = glob('launch/*.launch.py') + glob('launch/*.py')

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]
if launch_files:
    data_files.append(('share/' + package_name + '/launch', launch_files))

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryan',
    maintainer_email='you@example.com',
    description='Quadruped leg control (motor driver, joystick, and frequency controller)',
    license='MIT',
    license_files=('LICENSE',),  # keep if you created LICENSE; otherwise remove this line
    entry_points={
        'console_scripts': [
            'motor_node   = quad_legs.motor_node:main',
            'main_control = quad_legs.main_control:main',
            'quad_leg_ui  = quad_legs.quad_leg_ui:main',
            'quad_leg_freq = quad_legs.quad_leg_freq:main',  # include only if that file exists
            'beuhler_clock = quad_legs.beuhler_clock:main',
        ],
    },
)

