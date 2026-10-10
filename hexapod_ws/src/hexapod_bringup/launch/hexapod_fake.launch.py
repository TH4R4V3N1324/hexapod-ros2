import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    @brief Bring up the hexapod on ros2_control mock hardware, for testing without servos
    @return The launch description
    """
    bringup_path = get_package_share_directory('hexapod_bringup')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_path, 'launch', 'hexapod_robot.launch.py')),
            launch_arguments={'hardware': 'mock'}.items()
        ),
    ])
