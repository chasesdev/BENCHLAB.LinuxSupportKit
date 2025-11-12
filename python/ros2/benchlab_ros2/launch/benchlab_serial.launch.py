"""Launch file for BenchLab direct serial node (single device).

Usage:
    ros2 launch benchlab_ros2 benchlab_serial.launch.py device_path:=/dev/benchlab0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'device_path',
            default_value='/dev/benchlab0',
            description='Serial device path'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='Serial baudrate'
        ),
        DeclareLaunchArgument(
            'timeout',
            default_value='0.5',
            description='Command timeout in seconds'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='10.0',
            description='Telemetry publish rate in Hz'
        ),
        DeclareLaunchArgument(
            'device_namespace',
            default_value='',
            description='Optional namespace for topics (e.g., "device0")'
        ),

        # Launch serial node
        LifecycleNode(
            package='benchlab_ros2',
            executable='benchlab_serial_node',
            name='benchlab_serial_node',
            output='screen',
            parameters=[{
                'device_path': LaunchConfiguration('device_path'),
                'baudrate': LaunchConfiguration('baudrate'),
                'timeout': LaunchConfiguration('timeout'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'device_namespace': LaunchConfiguration('device_namespace'),
            }]
        ),
    ])
