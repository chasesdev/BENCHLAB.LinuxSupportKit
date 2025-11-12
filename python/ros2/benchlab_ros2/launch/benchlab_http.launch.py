"""Launch file for BenchLab HTTP bridge node (single device).

Usage:
    ros2 launch benchlab_ros2 benchlab_http.launch.py device_id:=benchlab0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'base_url',
            default_value='http://127.0.0.1:8080',
            description='Base URL of benchlabd HTTP service'
        ),
        DeclareLaunchArgument(
            'api_key',
            default_value='',
            description='API key for authentication (if required)'
        ),
        DeclareLaunchArgument(
            'device_id',
            default_value='benchlab0',
            description='Device ID (e.g., benchlab0)'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='10.0',
            description='Telemetry publish rate in Hz'
        ),
        DeclareLaunchArgument(
            'request_timeout',
            default_value='2.0',
            description='HTTP request timeout in seconds'
        ),
        DeclareLaunchArgument(
            'device_namespace',
            default_value='',
            description='Optional namespace for topics (e.g., "device0")'
        ),

        # Launch HTTP node
        LifecycleNode(
            package='benchlab_ros2',
            executable='benchlab_http_node',
            name='benchlab_http_node',
            output='screen',
            parameters=[{
                'base_url': LaunchConfiguration('base_url'),
                'api_key': LaunchConfiguration('api_key'),
                'device_id': LaunchConfiguration('device_id'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'request_timeout': LaunchConfiguration('request_timeout'),
                'device_namespace': LaunchConfiguration('device_namespace'),
            }]
        ),
    ])
