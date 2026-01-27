from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hexapod_hardware_interface',
            executable='fake_esp32_bridge_node',
            name='esp32_bridge',
            output='screen',
            parameters=[{
                'publish_rate': 100.0,
            }]
        ),
    ])