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
        executable='joy_node',
        name='joy_node',
        output='screen',
    )

    return LaunchDescription([
        controller_teleop_node,
        ros_joy_node
    ])