from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='rover_bringup',
            executable='rover_bringup',
            name='bringup_node',
            output='screen',
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
        ),
        Node(
            package='rover_station_sync',
            executable='fastapi_proxy_node',
            name='fastapi_proxy_node',
            output='screen',
        ),
    ])
