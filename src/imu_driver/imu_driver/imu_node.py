#!/usr/bin/env python3
"""
IMU Driver for Adafruit BNO055 9-DOF Absolute Orientation IMU
Publishes:
  - /imu/heading: std_msgs/Float32 (heading in radians, 0=East, CCW positive, ENU convention)
  - /imu/data: sensor_msgs/Imu (raw IMU data)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Header
import math
import time

# Hardware libraries are imported in _init_imu() to allow graceful degradation
# if hardware is not available


class IMUDriver(Node):
    def __init__(self):
        super().__init__('imu_driver')
        
        # Declare parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x28)
        self.declare_parameter('frame_id', 'imu')
        self.declare_parameter('publish_rate', 100.0)
        
        # Get parameters
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Publishers
        self.heading_publisher = self.create_publisher(Float32, '/imu/heading', 10)
        self.imu_publisher = self.create_publisher(Imu, '/imu/data', 10)
        
        # Initialize IMU
        self.imu = None
        self._init_imu()
        
        # Create timer for publishing at fixed rate
        self.create_timer(1.0 / self.publish_rate, self._timer_callback)
        
        # Calibration status
        self.last_calib_check = 0.0
        
        self.get_logger().info(f'IMU Driver initialized on I2C bus {self.i2c_bus}, address 0x{self.i2c_address:02x}')
    
    def _init_imu(self):
        """Initialize the BNO055 IMU"""
        try:
            # Import hardware libraries here (after node is initialized)
            import board
            import busio
            from adafruit_bno055 import Adafruit_BNO055
            
            # Create I2C bus
            i2c = busio.I2C(board.SCL, board.SDA)
            
            # Create sensor object
            self.imu = Adafruit_BNO055(address=self.i2c_address, i2c=i2c)
            
            # Wait for sensor to stabilize
            time.sleep(1)
            
            # Check if sensor is responding
            if self.imu.read_calibration_status():
                self.get_logger().info('BNO055 sensor initialized successfully')
            else:
                self.get_logger().warn('BNO055 sensor may not be responding, but continuing...')
            
        except ImportError as e:
            self.get_logger().error(f'Failed to import hardware libraries: {e}')
            self.get_logger().error('Install with: pip install adafruit-circuitpython-bno055 adafruit-circuitpython-busio')
            self.imu = None
        except Exception as e:
            self.get_logger().error(f'Failed to initialize IMU: {e}')
            self.get_logger().error('Make sure BNO055 is connected via I2C on the correct bus')
            self.imu = None
    
    def _timer_callback(self):
        """Timer callback to publish IMU data"""
        if self.imu is None:
            return
        
        try:
            # Read orientation data (Euler angles)
            euler = self.imu.euler
            if euler:
                heading_deg = euler[0]  # Yaw (heading)
                
                # Convert from degrees to radians
                # euler[0] is heading (0-360°, where 0° = North typically in geographic convention)
                # But BNO055 uses 0° = North, 90° = East in geographic navigation
                # We need to convert to 0° = East, CCW positive (ENU convention)
                # Geographic: 0°=N, 90°=E, 180°=S, 270°=W
                # ENU: 0°=E, 90°=N, 180°=W, 270°=S
                # Conversion: heading_enu = 90° - heading_geographic
                
                heading_enu_deg = 90.0 - heading_deg
                # Normalize to [-π, π]
                while heading_enu_deg > 180:
                    heading_enu_deg -= 360
                while heading_enu_deg < -180:
                    heading_enu_deg += 360
                
                heading_rad = math.radians(heading_enu_deg)
                
                # Publish heading
                heading_msg = Float32()
                heading_msg.data = heading_rad
                self.heading_publisher.publish(heading_msg)
            
            # Read accelerometer data
            accel = self.imu.accelerometer
            # Read gyroscope data
            gyro = self.imu.gyroscope
            # Read temperature
            temp = self.imu.temperature
            
            # Publish full IMU data
            if accel and gyro:
                imu_msg = Imu()
                imu_msg.header = Header()
                imu_msg.header.frame_id = self.frame_id
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                
                # Convert to ENU frame
                # Input: X=forward (North), Y=right (East), Z=up
                # ENU: X=East, Y=North, Z=up
                imu_msg.linear_acceleration.x = accel[1]  # East
                imu_msg.linear_acceleration.y = accel[0]  # North
                imu_msg.linear_acceleration.z = accel[2]  # Up
                
                # Gyroscope
                imu_msg.angular_velocity.x = gyro[1]  # East
                imu_msg.angular_velocity.y = gyro[0]  # North
                imu_msg.angular_velocity.z = gyro[2]  # Up
                
                # Default covariance values
                imu_msg.linear_acceleration_covariance = [0.0] * 9
                imu_msg.angular_velocity_covariance = [0.0] * 9
                
                self.imu_publisher.publish(imu_msg)
            
            # Periodically log calibration status
            current_time = time.time()
            if current_time - self.last_calib_check > 10.0:  # Check every 10 seconds
                self.last_calib_check = current_time
                try:
                    sys_calib, gyro_calib, accel_calib, mag_calib = self.imu.calibration_status()
                    self.get_logger().info(
                        f'Calibration: System={sys_calib}/3, Gyro={gyro_calib}/3, '
                        f'Accel={accel_calib}/3, Mag={mag_calib}/3'
                    )
                except:
                    pass
        
        except Exception as e:
            self.get_logger().error(f'Error reading IMU data: {e}')


def main(args=None):
    rclpy.init(args=args)
    imu_driver = IMUDriver()
    
    try:
        rclpy.spin(imu_driver)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
