from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    pkg_path = get_package_share_directory('hexapod_description')
    bridge_vision = LaunchConfiguration('bridge_vision')
    front_depth_width = LaunchConfiguration('front_depth_width')
    front_depth_height = LaunchConfiguration('front_depth_height')
    front_depth_rate = LaunchConfiguration('front_depth_rate')

    # Include the Gazebo simulation launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(Path(pkg_path) / "launch" / "gazebo.launch.py")]),
        launch_arguments={
            "use_sim_time": "true",
            "bridge_vision": bridge_vision,
            "front_depth_width": front_depth_width,
            "front_depth_height": front_depth_height,
            "front_depth_rate": front_depth_rate,
        }.items()
    )

    # Create a node for the hexapod gait controller
    hexapod_gait_controller = Node(
        package='hexapod_gait_controller',
        executable='gait_controller',
        name='hexapod_gait_controller',
        output='screen',
    )

    controller_teleop_node = Node(
        package='hexapod_teleop',
        executable='controller_teleop',
        name='controller_teleop',
        output='screen',
    )

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'bridge_vision',
            default_value='false',
            description='Bridge camera/depth topics from Gazebo (high bandwidth).'
        ),
        DeclareLaunchArgument(
            'front_depth_width',
            default_value='320',
            description='Front depth camera width in pixels.'
        ),
        DeclareLaunchArgument(
            'front_depth_height',
            default_value='240',
            description='Front depth camera height in pixels.'
        ),
        DeclareLaunchArgument(
            'front_depth_rate',
            default_value='6',
            description='Front depth camera update rate in Hz.'
        ),
        gazebo_launch,
        hexapod_gait_controller,
        controller_teleop_node,
        foxglove_bridge_node,
    ])