from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_pkg',
            namespace='rover_pkg',
            executable='rover',
            name='rover'
        ),
        Node(
            package='rover_pkg',
            namespace='rover_pkg',
            executable='monitor',
            name='monitor'
        ),
    ])