from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import ThisLaunchFileDir
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rover_node = Node(
        package='rover_pkg',
        executable='rover',
        parameters=[
            {'network_node': False},
        ]
    )

    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(
            get_package_share_directory('camera'), 'launch'),
            '/camera_node_cs.launch.py']
        )
    )

    return LaunchDescription([
        rover_node,
        included_launch
    ])