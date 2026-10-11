from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    controller_teleop_node = Node(
        package='hexapod_teleop',
        executable='controller_teleop',
        name='controller_teleop',
        output='screen',
    )

    ros_joy_node = Node(
        package='joy',
        # The game controller mapping reports the same layout for every supported gamepad (sticks
        # on axes 0-3, triggers on 4 and 5), which controller_teleop expects. Plain joy_node gave
        # the Steam Deck's raw order instead (triggers on axes 2 and 5).
        executable='game_controller_node',
        name='joy_node',
        output='screen',
    )

    return LaunchDescription([
        controller_teleop_node,
        ros_joy_node
    ])