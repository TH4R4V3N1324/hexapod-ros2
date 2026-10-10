import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    pkg_path = get_package_share_directory('hexapod_description')
    rviz_config = os.path.join(pkg_path, 'rviz', 'hexapod.rviz')
    ekf_file = os.path.join(get_package_share_directory('hexapod_bringup'), 'config', 'ekf.yaml')
    bridge_vision = LaunchConfiguration('bridge_vision')
    world = LaunchConfiguration('world')
    front_depth_width = LaunchConfiguration('front_depth_width')
    front_depth_height = LaunchConfiguration('front_depth_height')
    front_depth_rate = LaunchConfiguration('front_depth_rate')

    # Include the Gazebo simulation launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(Path(pkg_path) / "launch" / "gazebo.launch.py")]),
        launch_arguments={
            "use_sim_time": "true",
            "world": world,
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
        # Strafe mode when Nav2 runs, so it can move sideways (standard holonomic /cmd_vel)
        parameters=[{'use_sim_time': True}, {'initial_mode': ParameterValue(
            PythonExpression(["1 if '", LaunchConfiguration('nav'), "' == 'true' else 0"]), value_type=int)}]
    )

    controller_teleop_node = Node(
        package='hexapod_teleop',
        executable='controller_teleop',
        name='controller_teleop',
        output='screen',
    )

    # Fuses leg odometry and the simulated IMU gyro, publishes odom -> base_link
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_file, {'use_sim_time': True}]
    )

    # SLAM: builds a map from the lidar and publishes map -> odom
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': os.path.join(
                get_package_share_directory('hexapod_bringup'), 'config', 'slam_toolbox.yaml'),
            'use_sim_time': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('slam'))
    )

    # Nav2: plans and follows paths on the SLAM map, publishing /cmd_vel
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': os.path.join(
                get_package_share_directory('hexapod_bringup'), 'config', 'nav2_params.yaml'),
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav'))
    )

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'nav',
            default_value='false',
            description='Run Nav2 (use with slam:=true for a map).'
        ),
        DeclareLaunchArgument(
            'slam',
            default_value='false',
            description='Run slam_toolbox to build a map from the lidar.'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='hexapod_world',
            description='World file in hexapod_description/worlds, without .sdf (hexapod_world, empty_world, low_obstacles or tilt_platform).'
        ),
        DeclareLaunchArgument(
            'bridge_vision',
            default_value='false',
            description='Bridge camera/depth topics from Gazebo (high bandwidth).'
        ),
        DeclareLaunchArgument(
            'front_depth_width',
            default_value='240',
            description='Front depth camera width in pixels (Arducam ToF: 240).'
        ),
        DeclareLaunchArgument(
            'front_depth_height',
            default_value='180',
            description='Front depth camera height in pixels (Arducam ToF: 180).'
        ),
        DeclareLaunchArgument(
            'front_depth_rate',
            default_value='15',
            description='Front depth camera update rate in Hz (the real ToF runs at 30).'
        ),
        gazebo_launch,
        hexapod_gait_controller,
        controller_teleop_node,
        ekf_node,
        slam,
        nav,
        foxglove_bridge_node,
        rviz2_node,
    ])