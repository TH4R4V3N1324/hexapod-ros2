from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge_node',
        output='screen'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js1',
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }]
    )

    return LaunchDescription([
        foxglove_bridge_node,
        joy_node,
    ])