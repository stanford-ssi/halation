from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Launch arguments
    i2c_bus_arg = DeclareLaunchArgument(
        'i2c_bus',
        default_value='1',
        description='I2C bus number (1 for most RPi/Jetson, check with i2cdetect -y <bus>)'
    )
    
    i2c_address_arg = DeclareLaunchArgument(
        'i2c_address',
        default_value='0x28',
        description='I2C address of BNO055 (0x28 default, 0x29 if addr pin is high)'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100',
        description='IMU publish rate in Hz'
    )
    
    # Launch nodes
    imu_node = Node(
        package='imu_driver',
        executable='imu_node',
        name='imu_node',
        parameters=[
            {'i2c_bus': LaunchConfiguration('i2c_bus')},
            {'i2c_address': LaunchConfiguration('i2c_address')},
            {'frame_id': 'imu'},
            {'publish_rate': LaunchConfiguration('publish_rate')},
        ],
        output='screen',
    )
    
    return LaunchDescription([
        i2c_bus_arg,
        i2c_address_arg,
        publish_rate_arg,
        imu_node,
    ])
