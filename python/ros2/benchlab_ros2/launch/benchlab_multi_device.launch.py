"""Launch file for multiple BenchLab devices with namespaces.

This example launches 2 devices via direct serial access.
Topics will be published under device-specific namespaces:
- /device0/benchlab/telemetry
- /device1/benchlab/telemetry

Usage:
    ros2 launch benchlab_ros2 benchlab_multi_device.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    return LaunchDescription([
        # Device 0 (serial)
        LifecycleNode(
            package='benchlab_ros2',
            executable='benchlab_serial_node',
            name='benchlab_serial_node_0',
            output='screen',
            parameters=[{
                'device_path': '/dev/benchlab0',
                'baudrate': 115200,
                'timeout': 0.5,
                'publish_rate': 10.0,
                'device_namespace': 'device0',
            }]
        ),

        # Device 1 (serial)
        LifecycleNode(
            package='benchlab_ros2',
            executable='benchlab_serial_node',
            name='benchlab_serial_node_1',
            output='screen',
            parameters=[{
                'device_path': '/dev/benchlab1',
                'baudrate': 115200,
                'timeout': 0.5,
                'publish_rate': 10.0,
                'device_namespace': 'device1',
            }]
        ),

        # Note: For HTTP mode, use:
        # LifecycleNode(
        #     package='benchlab_ros2',
        #     executable='benchlab_http_node',
        #     name='benchlab_http_node_0',
        #     output='screen',
        #     parameters=[{
        #         'base_url': 'http://127.0.0.1:8080',
        #         'device_id': 'benchlab0',
        #         'publish_rate': 10.0,
        #         'device_namespace': 'device0',
        #     }]
        # ),
    ])
