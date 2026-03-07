from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge_node',
        output='screen'
    )

    return LaunchDescription([
        foxglove_bridge_node,
    ])