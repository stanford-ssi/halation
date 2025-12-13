import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sllidar_ros2'),
                'launch',
                'sllidar_a1_launch.py'
            )
        ),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'laser',
        }.items()
    )

    return LaunchDescription([
        Node(
            package='rover_sensing',
            executable='lidar_servo',
            name='lidar_servo',
            output='screen',
        ),
        Node(
            package='rover_sensing',
            executable='pointcloud_accumulator',
            name='pointcloud_accumulator',
            output='screen',
        ),
        sllidar_launch,
    ])
