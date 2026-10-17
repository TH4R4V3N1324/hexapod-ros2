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
    behaviour_file = os.path.join(get_package_share_directory('hexapod_bringup'), 'config', 'behaviour_manager.yaml')
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
            "gui": LaunchConfiguration('gui'),
            "spawn_x": LaunchConfiguration('spawn_x'),
            "spawn_y": LaunchConfiguration('spawn_y'),
            "spawn_z": LaunchConfiguration('spawn_z'),
            "spawn_yaw": LaunchConfiguration('spawn_yaw'),
        }.items()
    )

    # Create a node for the hexapod gait controller
    hexapod_gait_controller = Node(
        package='hexapod_gait_controller',
        executable='gait_controller',
        name='hexapod_gait_controller',
        output='screen',
        arguments=['--ros-args', '--log-level', ['hexapod_gait_controller:=', LaunchConfiguration('gait_log_level')]],
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

    # Watches the battery: warns when it's low and sits the robot down when it's critical
    behaviour_manager = Node(
        package='hexapod_control',
        executable='behaviour_manager',
        name='behaviour_manager',
        output='screen',
        parameters=[behaviour_file, {'use_sim_time': True}]
    )

    # Stands in for the daughter boards' power monitors. Set the battery voltage while running with
    # ros2 param set /sim_power_monitor voltage 9.8
    sim_power_monitor = Node(
        package='hexapod_control',
        executable='sim_power_monitor',
        name='sim_power_monitor',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Fuses leg odometry and the simulated IMU gyro, publishes odom -> base_link
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_file, {'use_sim_time': True}]
    )

    # Drops the robot's own legs from the lidar scan (/lidar/scan -> /lidar/scan_filtered),
    # which SLAM and Nav2 use
    lidar_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('hexapod_bringup'), 'config', 'laser_filters.yaml'), {'use_sim_time': True}],
        remappings=[('scan', '/lidar/scan'), ('scan_filtered', '/lidar/scan_filtered')]
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
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz'))
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
            description='World file in hexapod_description/worlds, without .sdf (hexapod_world, empty_world, low_obstacles, tilt_platform, rough_terrain, outdoor_slopes, outdoor_rocks or outdoor_low_friction).'
        ),
        DeclareLaunchArgument('gui', default_value='true', description='Show the Gazebo window (false runs the server only).'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz.'),
        DeclareLaunchArgument('gait_log_level', default_value='info',
                              description='gait_controller log level; debug logs each touchdown, stumble and ground search.'),
        DeclareLaunchArgument('spawn_x', default_value='0.0', description='Robot spawn x (m).'),
        DeclareLaunchArgument('spawn_y', default_value='0.0', description='Robot spawn y (m).'),
        DeclareLaunchArgument('spawn_z', default_value='0.5', description='Robot spawn height (m); it drops onto the ground.'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0', description='Robot spawn heading (rad).'),
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
        behaviour_manager,
        sim_power_monitor,
        ekf_node,
        lidar_filter,
        slam,
        nav,
        foxglove_bridge_node,
        rviz2_node,
    ])