#!/usr/bin/env python3
"""
GPS Driver for Adafruit Ultimate GPS Breakout (MTK3339/PA1616S)
Publishes:
  - /gps/fix: sensor_msgs/NavSatFix
  - /gps/vel: geometry_msgs/TwistWithCovarianceStamped
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped, Twist
from std_msgs.msg import Header
import serial
import threading
from datetime import datetime
import math

class GPSDriver(Node):
    def __init__(self):
        super().__init__('gps_driver')
        
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('frame_id', 'gps')
        
        # Get parameters
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Publishers
        self.fix_publisher = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.vel_publisher = self.create_publisher(TwistWithCovarianceStamped, '/gps/vel', 10)
        
        # Serial connection
        self.ser = None
        self.running = True
        
        # GPS data (with lock for thread safety)
        self.lock = threading.Lock()
        self.gps_data = {
            'latitude': 0.0,
            'longitude': 0.0,
            'altitude': 0.0,
            'velocity_north': 0.0,
            'velocity_east': 0.0,
            'hdop': 1.0,
            'vdop': 1.0,
            'has_fix': False,
            'timestamp': None,
        }
        
        # Covariance values (in meters^2)
        self.position_covariance = [9.0, 0.0, 0.0,  # lat
                                    0.0, 9.0, 0.0,  # lon
                                    0.0, 0.0, 9.0]  # alt
        self.velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Start serial reader thread
        self.reader_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self.reader_thread.start()
        
        self.get_logger().info(f'GPS Driver initialized on {self.port} @ {self.baudrate} baud')

    def _serial_reader(self):
        """Read and parse NMEA sentences from GPS module"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1.0)
            self.get_logger().info('Serial port opened successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.running = False
            return
        
        while self.running:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('ascii', errors='ignore').strip()
                    if line:
                        self._parse_nmea(line)
            except Exception as e:
                self.get_logger().warn(f'Serial read error: {e}')
    
    def _parse_nmea(self, sentence):
        """Parse NMEA sentences (GGA and VTG)"""
        if not sentence.startswith('$'):
            return
        
        try:
            # Remove checksum
            if '*' in sentence:
                sentence = sentence.split('*')[0]
            
            parts = sentence[1:].split(',')
            
            if len(parts) < 1:
                return
            
            sentence_type = parts[0]
            
            if sentence_type == 'GPGGA' or sentence_type == 'GNGGA':
                # GGA: Global Positioning System Fix Data
                # Format: $GPGGA,time,lat,N/S,lon,E/W,quality,num_sats,hdop,alt,M,geoid,M,dgps_age,dgps_id*checksum
                if len(parts) >= 10:
                    with self.lock:
                        try:
                            time_str = parts[1]
                            lat = float(parts[2]) if parts[2] else 0.0
                            lat_dir = parts[3]
                            lon = float(parts[4]) if parts[4] else 0.0
                            lon_dir = parts[5]
                            quality = int(parts[6]) if parts[6] else 0
                            num_sats = int(parts[7]) if parts[7] else 0
                            hdop = float(parts[8]) if parts[8] else 1.0
                            alt = float(parts[9]) if parts[9] else 0.0
                            
                            # Convert latitude and longitude from DDMM.MMMMM format to decimal
                            lat_deg = int(lat / 100)
                            lat_min = lat - lat_deg * 100
                            lat_decimal = lat_deg + lat_min / 60.0
                            if lat_dir == 'S':
                                lat_decimal = -lat_decimal
                            
                            lon_deg = int(lon / 100)
                            lon_min = lon - lon_deg * 100
                            lon_decimal = lon_deg + lon_min / 60.0
                            if lon_dir == 'W':
                                lon_decimal = -lon_decimal
                            
                            self.gps_data['latitude'] = lat_decimal
                            self.gps_data['longitude'] = lon_decimal
                            self.gps_data['altitude'] = alt
                            self.gps_data['hdop'] = hdop
                            self.gps_data['has_fix'] = quality > 0
                            self.gps_data['timestamp'] = datetime.now()
                            
                            # Update covariance based on DOP values
                            # Rough approximation: position_variance = 5.0 * hdop^2
                            self.position_covariance[0] = max(1.0, 5.0 * hdop * hdop)  # lat
                            self.position_covariance[4] = max(1.0, 5.0 * hdop * hdop)  # lon
                            self.position_covariance[8] = max(1.0, 10.0 * hdop * hdop)  # alt
                            
                            # Publish fix
                            self._publish_fix()
                        except (ValueError, IndexError) as e:
                            self.get_logger().debug(f'GGA parse error: {e}')
            
            elif sentence_type == 'GPVTG' or sentence_type == 'GNVTG':
                # VTG: Track and Ground Speed
                # Format: $GPVTG,track,T,track,M,speed_kts,N,speed_kmh,K*checksum
                if len(parts) >= 8:
                    with self.lock:
                        try:
                            speed_kts = float(parts[5]) if parts[5] else 0.0
                            # Convert knots to m/s: 1 knot = 0.51444 m/s
                            speed_ms = speed_kts * 0.51444
                            track = float(parts[1]) if parts[1] else 0.0
                            
                            # Convert track angle to velocity components (NED)
                            # Track: 0° = North, 90° = East
                            rad_track = math.radians(track)
                            self.gps_data['velocity_north'] = speed_ms * math.cos(rad_track)
                            self.gps_data['velocity_east'] = speed_ms * math.sin(rad_track)
                            
                            # Publish velocity
                            self._publish_vel()
                        except (ValueError, IndexError) as e:
                            self.get_logger().debug(f'VTG parse error: {e}')
        except Exception as e:
            self.get_logger().debug(f'NMEA parse error: {e}')
    
    def _publish_fix(self):
        """Publish NavSatFix message"""
        msg = NavSatFix()
        msg.header = Header()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.latitude = self.gps_data['latitude']
        msg.longitude = self.gps_data['longitude']
        msg.altitude = self.gps_data['altitude']
        msg.position_covariance = self.position_covariance
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        
        if self.gps_data['has_fix']:
            msg.status.status = NavSatFix.STATUS_FIX
        else:
            msg.status.status = NavSatFix.STATUS_NO_FIX
        
        self.fix_publisher.publish(msg)
    
    def _publish_vel(self):
        """Publish TwistWithCovarianceStamped message"""
        msg = TwistWithCovarianceStamped()
        msg.header = Header()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.twist.twist.linear.x = self.gps_data['velocity_east']
        msg.twist.twist.linear.y = self.gps_data['velocity_north']
        msg.twist.twist.linear.z = 0.0
        msg.twist.covariance = self.velocity_covariance
        
        self.vel_publisher.publish(msg)
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    gps_driver = GPSDriver()
    
    try:
        rclpy.spin(gps_driver)
    except KeyboardInterrupt:
        pass
    finally:
        gps_driver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
