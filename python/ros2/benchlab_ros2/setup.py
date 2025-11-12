from setuptools import setup
import os
from glob import glob

package_name = 'benchlab_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BenchLab Team',
    maintainer_email='support@benchlab.io',
    description='ROS2 nodes for BenchLab device integration',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'benchlab_http_node = benchlab_ros2.benchlab_http_node:main',
            'benchlab_serial_node = benchlab_ros2.benchlab_serial_node:main',
        ],
    },
)
