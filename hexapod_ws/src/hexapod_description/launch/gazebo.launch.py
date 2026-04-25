from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
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

    # ros_gz_bridge — replaces your joint_state_relay node
    # Bridges Gazebo clock to ROS so use_sim_time works
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_plugin_path,
        gz_resource_path,
        gazebo,
        robot_state_publisher,
        gz_bridge,
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