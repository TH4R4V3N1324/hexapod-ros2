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
        # The game controller mapping reports the same layout for every supported gamepad (sticks
        # on axes 0-3, triggers on 4 and 5), which controller_teleop expects. Plain joy_node gave
        # the Steam Deck's raw order instead (triggers on axes 2 and 5).
        executable='game_controller_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }]
    )

    return LaunchDescription([
        foxglove_bridge_node,
        joy_node,
    ])