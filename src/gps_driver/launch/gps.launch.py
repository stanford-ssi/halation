from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for GPS module'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='9600',
        description='Baudrate for GPS module'
    )
    
    # Launch nodes
    gps_node = Node(
        package='gps_driver',
        executable='gps_node',
        name='gps_node',
        parameters=[
            {'port': LaunchConfiguration('port')},
            {'baudrate': LaunchConfiguration('baudrate')},
            {'frame_id': 'gps'},
        ],
        output='screen',
    )
    
    return LaunchDescription([
        port_arg,
        baudrate_arg,
        gps_node,
    ])
