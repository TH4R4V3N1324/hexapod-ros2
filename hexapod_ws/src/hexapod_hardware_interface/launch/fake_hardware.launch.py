from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hexapod_hardware_interface',
            executable='hardware_interface_fake',
            name='hardware_interface_fake',
            output='screen',
            parameters=[{
                'publish_rate': 100.0,
            }]
        ),
    ])