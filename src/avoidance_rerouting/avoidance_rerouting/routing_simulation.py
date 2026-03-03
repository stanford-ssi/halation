import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class BoundingBox:
    """Class to store bounding box data for detected objects"""
    def __init__(self, height, width, distance, angle, rover_pos):
        """
        Parameters:
        - height: Height of the cube (z-dimension)
        - width: Width of the cube (perpendicular to distance vector)
        - distance: Distance from rover to object center (to the middle of width and height)
        - angle: Angle of the distance vector from rover's heading (degrees)
                 The width will be perpendicular to this angle
        - rover_pos: Tuple of (x, y, theta) for rover position
        """
        rover_x, rover_y, rover_theta = rover_pos
        
        # Store dimensions
        self.height = height
        self.width = width
        self.length = width  # Assuming square base, length = width
        
        # Convert angle to radians and make it relative to rover's heading
        # This is the angle of the distance vector
        distance_angle_rad = math.radians(angle) + rover_theta
        
        # Calculate global position (center of the box)
        self.x = rover_x + distance * math.cos(distance_angle_rad)
        self.y = rover_y + distance * math.sin(distance_angle_rad)
        
        # The box rotation: width is perpendicular to the distance vector
        # So the box's rotation angle is the distance angle
        self.angle = distance_angle_rad

class RoutingSimulation(Node):
    def __init__(self):
        super().__init__('routing_simulation')

        self.declare_parameter('use_test_obstacles', True)
        self.use_test_obstacles = self.get_parameter('use_test_obstacles').value

        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/object_bounding_boxes',
            10
        )

        self.rover_marker_publisher = self.create_publisher(
            Marker,
            '/rover_position',
            10
        )

        self.timer = self.create_timer(0.1, self.publish_markers)

        self.rover_x = -5.0
        self.rover_y = 0.0
        self.rover_theta = 0.0

        self.bounding_boxes = []
        self.blocking_bbox = None

        if self.use_test_obstacles:
            self.create_test_bounding_boxes()
        else:
            self.create_subscription(
                String,
                '/vision_detections',
                self.vision_detections_callback,
                10,
            )

        self.rover_width = 0.6
        self.rover_length = 0.8

        self.rover_safety_margin = .5 # (width or length)/safety_margin added to bounding box dimensions for collision checking

        # Target point (goal)
        self.target_x = 10.0
        self.target_y = 0.0
        self.target_threshold = 0.25

        # Motion parameters
        self.forward_step = 0.1
        self.lookahead_distance = 2.0
        
        self.get_logger().info('Routing Simulation Node Started')

    def define_obstacle(self, height, width, distance, angle):
        """
        Define and add a new obstacle bounding box.
        
        Parameters:
        - height: Height of the cube (z-dimension)
        - width: Width of the cube
        - distance: Distance from rover to object center
        - angle: Angle relative to rover's heading (degrees)
        """
        rover_pos = (self.rover_x, self.rover_y, self.rover_theta)
        
        bbox = BoundingBox(
            height=height,
            width=width,
            distance=distance,
            angle=angle,
            rover_pos=rover_pos
        )
        self.bounding_boxes.append(bbox)

    def vision_detections_callback(self, msg):
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Invalid JSON on /vision_detections')
            return

        self.bounding_boxes.clear()
        self.blocking_bbox = None

        for det in detections:
            self.define_obstacle(
                height=det.get('height_m', 0.5),
                width=det.get('width_m', 0.3),
                distance=det['distance_m'],
                angle=det['angle_deg'],
            )

        if detections:
            self.get_logger().info(f'Updated {len(detections)} vision obstacles')

    def create_test_bounding_boxes(self):
        """
        Create initial test bounding boxes for simulation.
        These are stored in self.bounding_boxes list.
        """
        orig_x = self.rover_x
        self.rover_theta = math.pi / 6
        self.rover_x = .5
        
        self.define_obstacle(1.0, 0.5, 3.0, -45)
        
        self.define_obstacle(.9, 0.6, 1.5, 30)

        self.define_obstacle(.9, 0.6, 4, 30)

        self.rover_x = -3
        self.define_obstacle(0.5, 1.0, 2.0, -70)

        self.rover_x = orig_x

    def create_bounding_box_marker(self, marker_id, bbox):
        """
        Create a bounding box marker for a detected object.
        
        Parameters:
        - marker_id: Unique ID for this marker
        - bbox: BoundingBox object with global position and dimensions
        
        Returns:
        - Marker object representing the bounding box
        """
        marker = Marker()
        marker.header.frame_id = "laser"  # Same frame as lidar
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "object_bounding_boxes"
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        
        # Use bbox global position
        obj_center_x = bbox.x
        obj_center_y = bbox.y
        obj_center_z = bbox.height / 2.0
        
        # Define the 8 corners of the cube in local frame
        # Local frame: length is along the distance direction, width is perpendicular
        half_width = bbox.width / 2.0
        half_length = bbox.length / 2.0
        half_height = bbox.height / 2.0
        
        # Corners in local object frame
        # Length (x-local) points along the distance vector from rover
        # Width (y-local) is perpendicular to the distance vector
        corners_local = [
            [-half_length, -half_width, -half_height],  # 0: bottom-back-left
            [half_length, -half_width, -half_height],   # 1: bottom-front-left
            [half_length, half_width, -half_height],    # 2: bottom-front-right
            [-half_length, half_width, -half_height],   # 3: bottom-back-right
            [-half_length, -half_width, half_height],   # 4: top-back-left
            [half_length, -half_width, half_height],    # 5: top-front-left
            [half_length, half_width, half_height],     # 6: top-front-right
            [-half_length, half_width, half_height],    # 7: top-back-right
        ]
        
        # Rotate corners around z-axis by bbox.angle and translate to global position
        corners_global = []
        cos_theta = math.cos(bbox.angle)
        sin_theta = math.sin(bbox.angle)
        
        for corner in corners_local:
            # Rotate in XY plane around z-axis
            x_rot = corner[0] * cos_theta - corner[1] * sin_theta
            y_rot = corner[0] * sin_theta + corner[1] * cos_theta
            
            # Translate to global position
            point = Point()
            point.x = obj_center_x + x_rot
            point.y = obj_center_y + y_rot
            point.z = obj_center_z + corner[2]
            corners_global.append(point)
        
        # Create edges for the bounding box (12 edges for a cube)
        edges = [
            # Bottom face
            (0, 1), (1, 2), (2, 3), (3, 0),
            # Top face
            (4, 5), (5, 6), (6, 7), (7, 4),
            # Vertical edges
            (0, 4), (1, 5), (2, 6), (3, 7)
        ]
        
        # Draw the 12 edges as solid lines
        # LINE_LIST requires pairs of points: [start1, end1, start2, end2, ...]
        marker.points = []
        for edge in edges:
            p1 = corners_global[edge[0]]
            p2 = corners_global[edge[1]]
            
            # Add start and end point of this edge
            marker.points.append(p1)
            marker.points.append(p2)
        
        # Set marker properties
        marker.scale.x = 0.02  # Line width
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        return marker

    def create_rover_marker(self):
        """
        Create a bounding box marker for the rover position.
        Rover is a 0.5m x 0.5m x 0.5m cube.
        
        Returns:
        - Marker object representing the rover
        """
        marker = Marker()
        marker.header.frame_id = "laser"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "rover"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        
        # Rover position
        rover_center_x = self.rover_x
        rover_center_y = self.rover_y
        rover_center_z = 0.25  # Half of 0.5m height
        
        # Define the 8 corners of the rover cube
        half_width = self.rover_width / 2.0
        half_length = self.rover_length / 2.0
        half_height = 0.25
        
        # Corners in local rover frame
        # Length (x-local) points along rover's heading
        # Width (y-local) is perpendicular to heading
        corners_local = [
            [-half_length, -half_width, -half_height],
            [half_length, -half_width, -half_height],
            [half_length, half_width, -half_height],
            [-half_length, half_width, -half_height],
            [-half_length, -half_width, half_height],
            [half_length, -half_width, half_height],
            [half_length, half_width, half_height],
            [-half_length, half_width, half_height],
        ]
        
        # Rotate corners around z-axis by rover heading
        corners_global = []
        cos_theta = math.cos(self.rover_theta)
        sin_theta = math.sin(self.rover_theta)
        
        for corner in corners_local:
            # Rotate in XY plane around z-axis
            x_rot = corner[0] * cos_theta - corner[1] * sin_theta
            y_rot = corner[0] * sin_theta + corner[1] * cos_theta
            
            # Translate to global position
            point = Point()
            point.x = rover_center_x + x_rot
            point.y = rover_center_y + y_rot
            point.z = rover_center_z + corner[2]
            corners_global.append(point)
        
        # Create edges for the bounding box (12 edges for a cube)
        edges = [
            # Bottom face
            (0, 1), (1, 2), (2, 3), (3, 0),
            # Top face
            (4, 5), (5, 6), (6, 7), (7, 4),
            # Vertical edges
            (0, 4), (1, 5), (2, 6), (3, 7)
        ]
        
        # Draw the 12 edges as solid lines
        marker.points = []
        for edge in edges:
            p1 = corners_global[edge[0]]
            p2 = corners_global[edge[1]]
            
            # Add start and end point of this edge
            marker.points.append(p1)
            marker.points.append(p2)
        
        # Set marker properties - different color for rover (blue)
        marker.scale.x = 0.02  # Line width
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        return marker

    def move_forward(self, d):
        """
        Move rover forward by distance d along its current heading.
        """
        self.rover_x += d * math.cos(self.rover_theta)
        self.rover_y += d * math.sin(self.rover_theta)

    def will_collide_ahead(self):
        """
        Stateful forward collision detection with hysteresis.
        """

        # If we already have a blocking obstacle,
        # check if it's still in the way with relaxed bounds
        if self.blocking_bbox is not None:
            bbox = self.blocking_bbox

            dx = bbox.x - self.rover_x
            dy = bbox.y - self.rover_y

            forward_dist = dx * math.cos(self.rover_theta) + dy * math.sin(self.rover_theta)
            lateral_dist = -dx * math.sin(self.rover_theta) + dy * math.cos(self.rover_theta)

            safe_forward = self.lookahead_distance + self.rover_length / self.rover_safety_margin + bbox.length / 2.0
            safe_lateral = self.rover_width / self.rover_safety_margin + bbox.width / 2.0

            # If still clearly blocking → remain blocked
            if 0 < forward_dist < safe_forward and abs(lateral_dist) < safe_lateral:
                return True

            # Otherwise obstacle cleared
            self.blocking_bbox = None

        # Normal detection
        for bbox in self.bounding_boxes:

            dx = bbox.x - self.rover_x
            dy = bbox.y - self.rover_y

            forward_dist = dx * math.cos(self.rover_theta) + dy * math.sin(self.rover_theta)
            lateral_dist = -dx * math.sin(self.rover_theta) + dy * math.cos(self.rover_theta)

            safe_forward = self.lookahead_distance + self.rover_length / self.rover_safety_margin + bbox.length / 2.0
            safe_lateral = self.rover_width / self.rover_safety_margin + bbox.width / 2.0

            if 0 < forward_dist < safe_forward:
                if abs(lateral_dist) < safe_lateral:
                    self.blocking_bbox = bbox
                    return True

        return False

    def compute_avoidance_heading(self):
        """
        Generate a new heading that avoids closest obstacle
        while still generally pointing toward target.
        """

        closest_bbox = None
        closest_dist = float('inf')

        for bbox in self.bounding_boxes:
            dx = bbox.x - self.rover_x
            dy = bbox.y - self.rover_y
            dist = math.hypot(dx, dy)

            if dist < closest_dist:
                closest_dist = dist
                closest_bbox = bbox

        if closest_bbox is None:
            return self.rover_theta
        if closest_bbox == self.blocking_bbox:
            return self.rover_theta

        # Determine which side obstacle is on
        dx = closest_bbox.x - self.rover_x
        dy = closest_bbox.y - self.rover_y

        lateral = -dx * math.sin(self.rover_theta) + dy * math.cos(self.rover_theta)

        # Steer away from obstacle
        turn_angle = math.radians(30)

        if lateral > 0:
            return self.rover_theta - turn_angle
        else:
            return self.rover_theta + turn_angle
        
    def update_rover_position(self):

        dx = self.target_x - self.rover_x
        dy = self.target_y - self.rover_y

        if (dx**2 + dy**2) < self.target_threshold:
            return

        # If currently blocked → stay in avoidance mode
        if self.blocking_bbox is not None:
            if self.will_collide_ahead():
                self.rover_theta = self.compute_avoidance_heading()
                self.move_forward(self.forward_step)
                return

        # Normal target tracking
        target_theta = math.atan2(dy, dx)

        heading_error = target_theta - self.rover_theta
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        self.rover_theta += 0.1 * heading_error

        if not self.will_collide_ahead():
            self.move_forward(self.forward_step)
        else:
            self.rover_theta = self.compute_avoidance_heading()
            self.move_forward(self.forward_step)
        
    def publish_markers(self):
        """
        Main publishing function that creates and publishes both
        rover position and bounding boxes.
        """
        # Update rover position based on navigation logic
        self.update_rover_position()

        # Publish rover position
        rover_marker = self.create_rover_marker()
        self.rover_marker_publisher.publish(rover_marker)
        
        # Publish bounding boxes
        marker_array = MarkerArray()
        
        # Create markers for all stored bounding boxes
        for i, bbox in enumerate(self.bounding_boxes):
            marker = self.create_bounding_box_marker(
                marker_id=i,
                bbox=bbox
            )
            marker_array.markers.append(marker)
        
        # Publish the marker array
        self.marker_publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = RoutingSimulation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()