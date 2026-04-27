from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bridge_vision = LaunchConfiguration('bridge_vision')
    front_depth_width = LaunchConfiguration('front_depth_width')
    front_depth_height = LaunchConfiguration('front_depth_height')
    front_depth_rate = LaunchConfiguration('front_depth_rate')

    pkg_path = get_package_share_directory('hexapod_description')
    pkg_share_parent = os.path.dirname(pkg_path)
    xacro_file = os.path.join(pkg_path, 'urdf', 'hexapod_primitives.urdf.xacro')
    controllers_file = os.path.join(pkg_path, 'config', 'controllers.yaml')
    world_file = os.path.join(pkg_path, 'worlds', 'hexapod_world.sdf')

    robot_description = ParameterValue(
        Command([
            'xacro ',
            xacro_file,
            ' ',
            'controllers_file:=',
            controllers_file,
            ' ',
            'front_depth_width:=',
            front_depth_width,
            ' ',
            'front_depth_height:=',
            front_depth_height,
            ' ',
            'front_depth_rate:=',
            front_depth_rate,
        ]),
        value_type=str
    )

    gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/jazzy/lib'
    )

    # Ensure Gazebo can resolve model://hexapod_description/... mesh URIs.
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_paths = [pkg_share_parent]
    if existing_resource_path:
        resource_paths.append(existing_resource_path)
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join(resource_paths)
    )

    # Gazebo Harmonic — replaces gazebo_ros gazebo.launch.py
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    joint_state_relay = Node(
        package="hexapod_description",
        executable="joint_state_relay",
        parameters=[{
            'use_sim_time': True
        }]
    )

    # Replaces gazebo_ros spawn_entity.py
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_hexapod',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'hexapod',
            '-x', '0',
            '-y', '0',
            '-z', '0.5'
        ],
        output='screen'
    )

    load_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    load_joint_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hexapod_joint_controller'],
        output='screen'
    )

    # Bridge core sim topics needed for control and lightweight visualization.
    gz_bridge_core = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/imu/mag@sensor_msgs/msg/MagneticField[gz.msgs.Magnetometer',
            '/front_depth/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/front_depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/front_depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen'
    )

    # High-bandwidth camera/depth bridges are optional to avoid Foxglove/RViz lag.
    gz_bridge_vision = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        condition=IfCondition(bridge_vision),
        arguments=[
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/front_depth/image@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'bridge_vision',
            default_value='false',
            description='Bridge camera/depth topics from Gazebo (high bandwidth).'
        ),
        DeclareLaunchArgument(
            'front_depth_width',
            default_value='640',
            description='Front depth camera width in pixels.'
        ),
        DeclareLaunchArgument(
            'front_depth_height',
            default_value='480',
            description='Front depth camera height in pixels.'
        ),
        DeclareLaunchArgument(
            'front_depth_rate',
            default_value='8',
            description='Front depth camera update rate in Hz.'
        ),
        gz_plugin_path,
        gz_resource_path,
        gazebo,
        robot_state_publisher,
        joint_state_relay,
        gz_bridge_core,
        gz_bridge_vision,
        spawn_entity,

        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_jsb]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=load_jsb,
                on_exit=[load_joint_controller]
            )
        ),
    ])