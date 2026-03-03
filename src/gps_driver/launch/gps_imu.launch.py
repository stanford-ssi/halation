from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # GPS Launch arguments
    gps_port_arg = DeclareLaunchArgument(
        'gps_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for GPS module'
    )
    
    # IMU Launch arguments
    i2c_bus_arg = DeclareLaunchArgument(
        'i2c_bus',
        default_value='1',
        description='I2C bus number'
    )
    
    i2c_address_arg = DeclareLaunchArgument(
        'i2c_address',
        default_value='0x28',
        description='I2C address of BNO055'
    )
    
    # GPS Node
    gps_node = Node(
        package='gps_driver',
        executable='gps_node',
        name='gps_node',
        parameters=[
            {'port': LaunchConfiguration('gps_port')},
            {'baudrate': 9600},
            {'frame_id': 'gps'},
        ],
        output='screen',
    )
    
    # IMU Node
    imu_node = Node(
        package='imu_driver',
        executable='imu_node',
        name='imu_node',
        parameters=[
            {'i2c_bus': LaunchConfiguration('i2c_bus')},
            {'i2c_address': LaunchConfiguration('i2c_address')},
            {'frame_id': 'imu'},
            {'publish_rate': 100.0},
        ],
        output='screen',
    )
    
    return LaunchDescription([
        gps_port_arg,
        i2c_bus_arg,
        i2c_address_arg,
        gps_node,
        imu_node,
    ])
