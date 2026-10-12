import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """
    @brief Bring up the hexapod on real servos (hardware:=st3215) or mock hardware (hardware:=mock)
    @return The launch description
    @note The IMU, power monitor and lidar drivers only start with the st3215 hardware.
    """
    hardware = LaunchConfiguration('hardware')
    serial_port = LaunchConfiguration('serial_port')
    update_rate = LaunchConfiguration('update_rate')

    description_path = get_package_share_directory('hexapod_description')
    hardware_path = get_package_share_directory('hexapod_hardware_interface')
    xacro_file = os.path.join(description_path, 'urdf', 'hexapod.urdf.xacro')
    controllers_file = os.path.join(description_path, 'config', 'controllers.yaml')
    sensors_file = os.path.join(hardware_path, 'config', 'sensors.yaml')
    ekf_file = os.path.join(get_package_share_directory('hexapod_bringup'), 'config', 'ekf.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' hardware:=', hardware, ' serial_port:=', serial_port]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # controllers.yaml runs at 1000 Hz for Gazebo, the servo bus needs a much lower rate
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[controllers_file, {'update_rate': ParameterValue(update_rate, value_type=int)}],
        remappings=[('~/robot_description', '/robot_description')]
    )

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', 'hexapod_joint_controller'],
        output='screen'
    )

    joint_state_relay = Node(
        package='hexapod_description',
        executable='joint_state_relay',
        output='screen'
    )

    gait_controller = Node(
        package='hexapod_gait_controller',
        executable='gait_controller',
        name='hexapod_gait_controller',
        output='screen',
        # Strafe mode when Nav2 runs, so it can move sideways (standard holonomic /cmd_vel)
        parameters=[{'initial_mode': ParameterValue(
            PythonExpression(["1 if '", LaunchConfiguration('nav'), "' == 'true' else 0"]), value_type=int)}]
    )

    controller_teleop = Node(
        package='hexapod_teleop',
        executable='controller_teleop',
        name='controller_teleop',
        output='screen'
    )

    # Fuses leg odometry and the IMU gyro, publishes odom -> base_link
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_file]
    )

    # SLAM: builds a map from the lidar and publishes map -> odom
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': os.path.join(
                get_package_share_directory('hexapod_bringup'), 'config', 'slam_toolbox.yaml'),
            'use_sim_time': 'false',
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
            'use_sim_time': 'false',
            'autostart': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav'))
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen'
    )

    sensors = GroupAction(
        condition=IfCondition(PythonExpression(["'", hardware, "' == 'st3215'"])),
        actions=[
            Node(
                package='hexapod_hardware_interface',
                executable='ism330dhcx_node',
                name='ism330dhcx',
                output='screen',
                parameters=[sensors_file]
            ),
            # Fuses imu/data_raw into an orientation estimate on imu/data
            Node(
                package='imu_filter_madgwick',
                executable='imu_filter_madgwick_node',
                name='imu_filter',
                output='screen',
                parameters=[sensors_file]
            ),
            Node(
                package='hexapod_hardware_interface',
                executable='ina226_node',
                name='ina226',
                output='screen',
                parameters=[sensors_file]
            ),
            # RPLIDAR A2M12, published on /lidar/scan like the simulated lidar
            Node(
                package='rplidar_ros',
                executable='rplidar_composition',
                name='rplidar',
                output='screen',
                parameters=[sensors_file],
                remappings=[('scan', '/lidar/scan')]
            ),
            # Drops the robot's own legs from the scan (/lidar/scan -> /lidar/scan_filtered),
            # which SLAM and Nav2 use
            Node(
                package='laser_filters',
                executable='scan_to_scan_filter_chain',
                name='scan_to_scan_filter_chain',
                output='screen',
                parameters=[os.path.join(get_package_share_directory('hexapod_bringup'), 'config', 'laser_filters.yaml')],
                remappings=[('scan', '/lidar/scan'), ('scan_filtered', '/lidar/scan_filtered')]
            ),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware',
            default_value='st3215',
            description='ros2_control hardware: st3215 (real servos) or mock.'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyAMA0',
            description='Serial device for the servo bus.'
        ),
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
            'update_rate',
            default_value='100',
            description='Controller manager update rate in Hz.'
        ),
        robot_state_publisher,
        controller_manager,
        controller_spawner,
        joint_state_relay,
        gait_controller,
        controller_teleop,
        ekf,
        slam,
        nav,
        foxglove_bridge,
        sensors,
    ])
