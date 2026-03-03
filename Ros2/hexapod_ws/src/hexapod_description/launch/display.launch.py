from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('hexapod_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'hexapod.urdf.xacro')
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'hexapod.rviz')

    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='hexapod_hardware_interface',
            executable='esp_bridge_fake',
            name='esp32_bridge',
            output='screen',
            parameters=[{
                'publish_rate': 100.0,
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])