import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class MockLidarPublisher(Node):
    def __init__(self):
        super().__init__('mock_lidar_publisher')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)
        
        # Laser parameters
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = math.radians(1.0) # 1 degree resolution
        
        # Animation state
        self.obstacle_angle = 0.0
        self.direction = 1

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.range_min = 0.1
        scan.range_max = 20.0

        # Calculate number of points
        num_points = int((self.angle_max - self.angle_min) / self.angle_increment)
        
        # 1. Background: Far away wall (smooth circle at 10m)
        ranges = np.full(num_points, 10.0)
        
        # 2. Add a Moving Obstacle (Smooth "Blob")
        # We simulate a solid object about 1 meter wide
        obs_width_rad = math.radians(30) 
        obs_dist = 1.5 # 1.5 meters away
        
        # Animate obstacle angle
        self.obstacle_angle += 0.05 * self.direction
        if self.obstacle_angle > 1.0 or self.obstacle_angle < -1.0:
            self.direction *= -1

        # Fill in the obstacle points
        start_angle = self.obstacle_angle - (obs_width_rad / 2)
        end_angle = self.obstacle_angle + (obs_width_rad / 2)
        
        start_idx = int((start_angle - self.angle_min) / self.angle_increment)
        end_idx = int((end_angle - self.angle_min) / self.angle_increment)

        # Clamp indices
        start_idx = max(0, start_idx)
        end_idx = min(num_points, end_idx)

        for i in range(start_idx, end_idx):
            # Give it a slight curve so it looks like a real object, not a flat line
            # Simple parabolic curve for "roundness"
            offset = i - start_idx
            width = end_idx - start_idx
            curve = math.sin((offset / width) * math.pi) * 0.2
            ranges[i] = obs_dist - curve # Object is closer than background

        scan.ranges = ranges.tolist()
        self.publisher_.publish(scan)
        # self.get_logger().info(f'Published scan with obstacle at {self.obstacle_angle:.2f} rad')

def main(args=None):
    rclpy.init(args=args)
    node = MockLidarPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()