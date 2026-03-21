from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
 
def generate_launch_description():
    pkg_path = get_package_share_directory('hexapod_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'hexapod_primitives.urdf.xacro')
    controllers_file = os.path.join(pkg_path, 'config', 'controllers.yaml')
 
    # Process the URDF file
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
 
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        )
    )
 
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )
 
    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_hexapod',
        arguments=['-topic', 'robot_description',
                   '-entity', 'hexapod',
                   '-x', '0',
                   '-y', '0',
                   '-z', '0.5'],
        output='screen'
    )
 
    # Load joint_state_broadcaster after spawn completes
    load_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
 
    # Load hexapod_joint_controller after joint_state_broadcaster is loaded
    load_joint_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'hexapod_joint_controller'],
        output='screen'
    )
 
    # Relay node: JointState -> Float64MultiArray (sim only, not used on real hardware)
    joint_state_relay = Node(
        package='hexapod_description',
        executable='joint_state_relay',
        name='joint_state_relay',
        parameters=[{'use_sim_time': True}]
    )
 
    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        spawn_entity,
 
        # Chain controller loading after spawn
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_jsb]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=load_jsb,
                on_exit=[load_joint_controller, joint_state_relay]
            )
        ),
    ])