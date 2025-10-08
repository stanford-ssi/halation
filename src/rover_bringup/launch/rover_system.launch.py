from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_bringup',
            executable='rover_bringup',
            name='bringup_node',
            output='screen',
        ),
    ])
