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
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = math.radians(1)
        self.range_min = 0.1
        self.range_max = 12.0
        self.obstacle_angle = 0.0

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        num_points = int((self.angle_max - self.angle_min) / self.angle_increment)
        ranges = np.random.uniform(10.0, 12.0, num_points)
        obs_idx = int((self.obstacle_angle - self.angle_min) / self.angle_increment)
        for i in range(obs_idx - 5, obs_idx + 5):
            if 0 <= i < num_points:
                ranges[i] = np.random.uniform(1.0, 2.0)
        scan.ranges = ranges.tolist()
        self.publisher_.publish(scan)
        self.get_logger().info('Published mock scan')

def main(args=None):
    rclpy.init(args=args)
    node = MockLidarPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
