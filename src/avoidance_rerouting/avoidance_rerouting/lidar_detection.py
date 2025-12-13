"""
Takes raw LIDAR data and detects obstacles within 2 meters, publishing their positions to /obstacle topic.
Rerouting node will subscribe to /obstacle to get obstacle positions and use that information to reroute rover.
V1 only detects points, without clustering to identify distinct objects.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
import math

class LidarDetectionNodeV1(Node):
    """
    Detects obstacles within 2 meters using LIDAR data and publishes their positions. Only detects points, doesnt cluster to detect objects.
    """
    def __init__(self):
        super().__init__("LidarDetectionNode")
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(PointStamped, '/obstacle', 10)
        self.threshold = 2.0  # meters
        self.get_logger().info('Lidar detection node started')

    def scan_callback(self, scan: LaserScan):
        # Find the minimum range within 2 m
        min_range = min(scan.ranges)
        if min_range < self.threshold:
            i = scan.ranges.index(min_range)
            angle = scan.angle_min + i * scan.angle_increment
            # Convert to Cartesian coordinates (in laser frame)
            x = min_range * math.cos(angle)
            y = min_range * math.sin(angle)
            msg = PointStamped()
            msg.header.frame_id = 'laser'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.point.x, msg.point.y, msg.point.z = x, y, 0.0
            self.pub.publish(msg)
            self.get_logger().info(f'Obstacle at ({x:.2f}, {y:.2f}) m')

def main(args=None):
    rclpy.init(args=args)
    node = LidarDetectionNodeV1()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
