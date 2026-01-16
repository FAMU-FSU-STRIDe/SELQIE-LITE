from setuptools import setup, find_packages

package_name = 'latch'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/latch.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'pyserial',
    ],
    zip_safe=True,
    maintainer='SELQIE',
    maintainer_email='',
    description='Serial driver node for the Hitec D954SW latch controller',
    license='MIT',
    entry_points={
        'console_scripts': [
            'latch_node = latch.latch_node:main',
        ],
    },
)
