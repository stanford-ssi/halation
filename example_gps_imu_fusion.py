#!/usr/bin/env python3
"""
Example: GPS and IMU data fusion for autonomous rover navigation.
This demonstrates how to use the GPS and IMU data in your rover code.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from geometry_msgs.msg import TwistWithCovarianceStamped
import math
from dataclasses import dataclass
from typing import Optional
from datetime import datetime


@dataclass
class RoverState:
    """Represents the current state of the rover"""
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    heading_rad: float = 0.0
    velocity_north: float = 0.0  # m/s
    velocity_east: float = 0.0   # m/s
    position_covariance: float = 0.0
    has_gps_fix: bool = False
    last_update: Optional[datetime] = None
    
    def heading_deg(self) -> float:
        """Convert heading from radians (ENU) to degrees"""
        return math.degrees(self.heading_rad)
    
    def heading_geographic(self) -> float:
        """Convert ENU heading to geographic heading (0°=North, CW positive)"""
        # ENU: 0=East, π/2=North, CCW positive
        # Geographic: 0=North, 90=East, CW positive
        # Conversion: heading_geo = 90° - heading_enu
        heading_geo_rad = math.pi/2 - self.heading_rad
        heading_geo_deg = math.degrees(heading_geo_rad)
        # Normalize to [0, 360)
        while heading_geo_deg < 0:
            heading_geo_deg += 360
        while heading_geo_deg >= 360:
            heading_geo_deg -= 360
        return heading_geo_deg
    
    def speed_ms(self) -> float:
        """Calculate total velocity magnitude in m/s"""
        return math.sqrt(self.velocity_north**2 + self.velocity_east**2)
    
    def speed_kmh(self) -> float:
        """Calculate total velocity in km/h"""
        return self.speed_ms() * 3.6


class GPSIMUFusionNode(Node):
    """
    Example node that fuses GPS and IMU data for rover navigation.
    Demonstrates how to use the published topics from gps_driver and imu_driver.
    """
    
    def __init__(self):
        super().__init__('gps_imu_fusion_example')
        
        # Create state object
        self.rover_state = RoverState()
        
        # Subscribe to GPS fix
        self.gps_fix_sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self._gps_fix_callback,
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        
        # Subscribe to GPS velocity
        self.gps_vel_sub = self.create_subscription(
            TwistWithCovarianceStamped,
            '/gps/vel',
            self._gps_vel_callback,
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        
        # Subscribe to IMU heading
        self.imu_heading_sub = self.create_subscription(
            Float32,
            '/imu/heading',
            self._imu_heading_callback,
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        
        # Create a timer to print state at regular intervals
        self.timer = self.create_timer(2.0, self._print_state)
        
        self.get_logger().info('GPS/IMU Fusion Node initialized')
        self.get_logger().info('Waiting for sensor data...')
    
    def _gps_fix_callback(self, msg: NavSatFix):
        """Handle incoming GPS fix messages"""
        self.rover_state.latitude = msg.latitude
        self.rover_state.longitude = msg.longitude
        self.rover_state.altitude = msg.altitude
        
        # Check for valid fix
        self.rover_state.has_gps_fix = (msg.status.status >= 1)
        
        # Extract covariance (position variance)
        if msg.position_covariance and len(msg.position_covariance) >= 9:
            # Approximate position uncertainty (standard deviation of latitude)
            self.rover_state.position_covariance = math.sqrt(msg.position_covariance[0])
        
        self.rover_state.last_update = datetime.now()
        
        if self.rover_state.has_gps_fix:
            self.get_logger().debug(
                f'GPS Fix: {msg.latitude:.6f}, {msg.longitude:.6f}, '
                f'Alt: {msg.altitude:.1f}m'
            )
    
    def _gps_vel_callback(self, msg: TwistWithCovarianceStamped):
        """Handle incoming GPS velocity messages"""
        # Velocity in ENU frame
        self.rover_state.velocity_east = msg.twist.twist.linear.x   # m/s
        self.rover_state.velocity_north = msg.twist.twist.linear.y  # m/s
        
        speed = self.rover_state.speed_ms()
        self.get_logger().debug(f'GPS Velocity: {speed:.2f} m/s')
    
    def _imu_heading_callback(self, msg: Float32):
        """Handle incoming IMU heading messages"""
        self.rover_state.heading_rad = float(msg.data)
        
        # Keep heading in [-π, π] range
        while self.rover_state.heading_rad > math.pi:
            self.rover_state.heading_rad -= 2 * math.pi
        while self.rover_state.heading_rad <= -math.pi:
            self.rover_state.heading_rad += 2 * math.pi
    
    def _print_state(self):
        """Periodically print the current rover state"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('ROVER STATE:')
        self.get_logger().info('=' * 60)
        
        # Position
        if self.rover_state.has_gps_fix:
            self.get_logger().info(
                f'Position: {self.rover_state.latitude:.6f}°, '
                f'{self.rover_state.longitude:.6f}°'
            )
            self.get_logger().info(f'Altitude: {self.rover_state.altitude:.1f} m')
            self.get_logger().info(
                f'Position Uncertainty: ±{self.rover_state.position_covariance:.1f} m'
            )
        else:
            self.get_logger().warn('GPS: NO FIX')
        
        # Heading
        heading_enu_deg = self.rover_state.heading_deg()
        heading_geo_deg = self.rover_state.heading_geographic()
        self.get_logger().info(
            f'Heading (ENU): {heading_enu_deg:.1f}° (0°=East, CCW+)'
        )
        self.get_logger().info(
            f'Heading (Geographic): {heading_geo_deg:.1f}° (0°=North, CW+)'
        )
        
        # Velocity
        speed = self.rover_state.speed_ms()
        speed_kmh = self.rover_state.speed_kmh()
        self.get_logger().info(
            f'Velocity: {speed:.2f} m/s ({speed_kmh:.1f} km/h)'
        )
        self.get_logger().info(
            f'  North: {self.rover_state.velocity_north:+.2f} m/s'
        )
        self.get_logger().info(
            f'  East:  {self.rover_state.velocity_east:+.2f} m/s'
        )
        
        self.get_logger().info('=' * 60 + '\n')
    
    def calculate_distance_to_waypoint(self, target_lat: float, target_lon: float) -> float:
        """
        Calculate distance to a GPS waypoint using Haversine formula.
        
        Args:
            target_lat: Target latitude in degrees
            target_lon: Target longitude in degrees
            
        Returns:
            Distance in meters
        """
        if not self.rover_state.has_gps_fix:
            self.get_logger().warn('No GPS fix, cannot calculate distance')
            return float('inf')
        
        # Earth's radius in meters
        R = 6371000
        
        # Convert to radians
        lat1 = math.radians(self.rover_state.latitude)
        lon1 = math.radians(self.rover_state.longitude)
        lat2 = math.radians(target_lat)
        lon2 = math.radians(target_lon)
        
        # Haversine formula
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        
        distance = R * c
        return distance
    
    def calculate_bearing_to_waypoint(self, target_lat: float, target_lon: float) -> float:
        """
        Calculate bearing to a GPS waypoint.
        
        Args:
            target_lat: Target latitude in degrees
            target_lon: Target longitude in degrees
            
        Returns:
            Bearing in degrees (0°=North, CW positive, geographic convention)
        """
        if not self.rover_state.has_gps_fix:
            self.get_logger().warn('No GPS fix, cannot calculate bearing')
            return 0.0
        
        # Convert to radians
        lat1 = math.radians(self.rover_state.latitude)
        lon1 = math.radians(self.rover_state.longitude)
        lat2 = math.radians(target_lat)
        lon2 = math.radians(target_lon)
        
        # Calculate bearing
        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        
        bearing_rad = math.atan2(x, y)
        bearing_deg = math.degrees(bearing_rad)
        
        # Normalize to [0, 360)
        if bearing_deg < 0:
            bearing_deg += 360
        
        return bearing_deg
    
    def calculate_heading_error(self, target_bearing: float) -> float:
        """
        Calculate heading error from current heading to target bearing.
        
        Args:
            target_bearing: Target bearing in degrees (geographic, 0°=North, CW+)
            
        Returns:
            Heading error in degrees (positive = turn CW, negative = turn CCW)
        """
        current_heading = self.rover_state.heading_geographic()
        
        # Calculate error
        error = target_bearing - current_heading
        
        # Normalize to [-180, 180]
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        
        return error


def main(args=None):
    rclpy.init(args=args)
    
    # Create fusion node
    fusion_node = GPSIMUFusionNode()
    
    # Example: Calculate distance to a waypoint
    # (This would normally come from your navigation planner)
    target_lat = 37.425
    target_lon = -122.142
    
    # Spin the node
    try:
        rclpy.spin(fusion_node)
    except KeyboardInterrupt:
        pass
    finally:
        fusion_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
