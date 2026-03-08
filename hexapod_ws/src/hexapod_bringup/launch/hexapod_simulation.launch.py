from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the URDF file
    pkg_path = get_package_share_directory('hexapod_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'hexapod.urdf.xacro')
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'hexapod.rviz')

    robot_description = Command(['xacro ', xacro_file])

    # Create a node to publish the robot state
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    hardware_interface_fake_node = Node(
        package='hexapod_hardware_interface',
        executable='hardware_interface_fake',
        name='hardware_interface_fake',
        output='screen',
        parameters=[{
            'publish_rate': 100.0,
        }]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    hexapod_gait_controller = Node(
        package='hexapod_gait_controller',
        executable='gait_controller',
        name='hexapod_gait_controller',
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher_node,
        hardware_interface_fake_node,
        rviz2_node,
        hexapod_gait_controller
    ])