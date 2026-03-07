from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration

import urllib.parse


def _launch_foxglove_studio(context):
    bridge_url = LaunchConfiguration('foxglove_bridge_url').perform(context).strip()
    layout_id = LaunchConfiguration('foxglove_layout_id').perform(context).strip()

    query = {
        'ds': 'foxglove-websocket',
        'ds.url': bridge_url,
    }
    if layout_id:
        query['layoutId'] = layout_id

    deep_link = f"foxglove://open?{urllib.parse.urlencode(query)}"
    return [ExecuteProcess(cmd=['foxglove-studio', deep_link], output='screen')]

def generate_launch_description():
    foxglove_bridge_url_arg = DeclareLaunchArgument(
        'foxglove_bridge_url',
        default_value='ws://localhost:8765',
        description='Foxglove WebSocket URL to open in Foxglove Studio.'
    )

    foxglove_layout_id_arg = DeclareLaunchArgument(
        'foxglove_layout_id',
        default_value='',
        description='Optional Foxglove layoutId to auto-open a saved layout.'
    )

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge_node',
        output='screen'
    )

    foxglove_bridge_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=foxglove_bridge_node,
            on_start=[
                OpaqueFunction(function=_launch_foxglove_studio)
            ]
        )
    )

    return LaunchDescription([
        foxglove_bridge_url_arg,
        foxglove_layout_id_arg,
        foxglove_bridge_node,
        foxglove_bridge_event_handler,
    ])