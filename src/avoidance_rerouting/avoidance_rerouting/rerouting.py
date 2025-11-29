# Rerouting Node (takes segmented LIDAR output and generates a new path to avoid obstacles)
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from slg_msgs.msg import SegmentArray
import json

class ReroutingNode(Node):
    def __init__(self):
        super().__init__('rerouting_node')
        
        # Subscribe to the processed segments (green boxes)
        self.subscription = self.create_subscription(
            SegmentArray,
            '/segments',
            self.listener_callback,
            10)
            
        # Publish commands to the motor interface
        self.publisher_ = self.create_publisher(String, '/motor_command', 10)
        
        # Safety Parameters
        self.stop_distance = 1.5  # Meters: How close can an object get?
        self.lane_width = 0.5     # Meters: How wide is our path (center to edge)?

        self.get_logger().info("Rerouting Node (V2) Initialized")

    def listener_callback(self, msg):
        action = "forwards"
        
        # Loop through every object the segmentation node found
        for segment in msg.segments:
            # Calculate the centroid (geometric center) of the object
            if not segment.points:
                continue
                
            avg_x = sum(p.x for p in segment.points) / len(segment.points)
            avg_y = sum(p.y for p in segment.points) / len(segment.points)

            # Decision Logic: Is this object in my way?
            # X is forward distance, Y is left/right offset
            if 0 < avg_x < self.stop_distance:  # Is it close?
                if abs(avg_y) < self.lane_width: # Is it in my lane?
                    self.get_logger().warn(f"Avoidance Triggered! Object {segment.id} at ({avg_x:.2f}, {avg_y:.2f})")
                    
                    # Simple Avoidance: Steer away from the center of the object
                    if avg_y > 0:
                        action = "turn_right" # Object is on left -> go right
                    else:
                        action = "turn_left"  # Object is on right -> go left
                    
                    # Once we find one threat, we react and stop checking others
                    break

        self.publish_command(action)

    def publish_command(self, command):
        msg = String()
        msg.data = json.dumps({"command": command})
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ReroutingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

