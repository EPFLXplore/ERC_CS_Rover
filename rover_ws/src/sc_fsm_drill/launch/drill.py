from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='sc_fsm_drill',
                namespace='sc_fsm_drill',
                executable='drill',
            )
        ]
    )