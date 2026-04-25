from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
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

    # Create a node to simulate the hardware interface
    hardware_interface_fake_node = Node(
        package='hexapod_hardware_interface',
        executable='hardware_interface_fake',
        name='hardware_interface_fake',
        output='screen',
        parameters=[{
            'publish_rate': 100.0,
        }]
    )

    # Include the Gazebo simulation launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(Path(pkg_path) / "launch" / "gazebo.launch.py")]),
        launch_arguments={"use_sim_time": "true"}.items()
    )

    # Create a node for the hexapod gait controller
    hexapod_gait_controller = Node(
        package='hexapod_gait_controller',
        executable='gait_controller',
        name='hexapod_gait_controller',
        output='screen',
    )

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher_node,
        hardware_interface_fake_node,
        gazebo_launch,
        hexapod_gait_controller,
        foxglove_bridge_node,
    ])