from glob import glob
from setuptools import setup

package_name = 'reed_switch'

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
    description='Reed switch monitor node using Jetson GPIO.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'reed_switch_node = reed_switch.reed_switch_node:main',
        ],
    },
)
