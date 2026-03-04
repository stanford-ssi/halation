import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
import math

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
        
        # Publisher for bounding box markers
        self.marker_publisher = self.create_publisher(
            MarkerArray, 
            '/object_bounding_boxes', 
            10
        )
        
        # Publisher for rover position marker
        self.rover_marker_publisher = self.create_publisher(
            Marker,
            '/rover_position',
            10
        )

        # Publisher for planned trajectory
        self.trajectory_publisher = self.create_publisher(
            Marker,
            '/planned_trajectory',
            10
        )
        
        # Timer to publish at regular intervals
        self.timer = self.create_timer(0.2, self.publish_markers)
        
        # Rover position (x, y, theta)
        self.rover_x = -5.0
        self.rover_y = 0.0
        self.rover_theta = 0.0  # Rover's heading angle (radians)
        
        # List to store all detected bounding boxes
        self.bounding_boxes = []
        self.blocking_bbox = None

        # Create test bounding boxes
        self.create_test_bounding_boxes()

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

        # Trajectory planning parameters
        self.max_plan_steps = 300     # Max simulation steps per plan
        self.plan_turn_angle = math.radians(5)  # Turn increment when avoiding
        
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

    # -----------------------------------------------------------------------
    # Pure geometry helpers (operate on explicit state, no self mutation)
    # -----------------------------------------------------------------------

    def _will_collide(self, vx, vy, vtheta, bbox):
        """
        Check whether a virtual rover at (vx, vy, vtheta) would collide with
        bbox within the lookahead window.  Pure function — no side-effects.
        """
        dx = bbox.x - vx
        dy = bbox.y - vy

        forward_dist = dx * math.cos(vtheta) + dy * math.sin(vtheta)
        lateral_dist = -dx * math.sin(vtheta) + dy * math.cos(vtheta)

        safe_forward = (
            self.lookahead_distance
            + self.rover_length / self.rover_safety_margin
            + bbox.length / 2.0
        )
        safe_lateral = (
            self.rover_width / self.rover_safety_margin
            + bbox.width / 2.0
        )

        return (0 < forward_dist < safe_forward) and (abs(lateral_dist) < safe_lateral)

    def _any_collision(self, vx, vy, vtheta):
        """Return the first blocking bbox for a virtual pose, or None."""
        for bbox in self.bounding_boxes:
            if self._will_collide(vx, vy, vtheta, bbox):
                return bbox
        return None

    def _avoidance_heading(self, vx, vy, vtheta, blocking_bbox):
        """
        Compute an avoidance heading given the virtual rover state and the
        obstacle that is currently blocking.  Pure function — no side-effects.

        The logic mirrors compute_avoidance_heading() but uses the nearest
        *non-blocking* obstacle to decide which side to steer toward, so the
        virtual rover steers away from clutter on the open side.
        """
        closest_bbox = None
        closest_dist = float('inf')

        for bbox in self.bounding_boxes:
            if bbox is blocking_bbox:
                continue
            dist = math.hypot(bbox.x - vx, bbox.y - vy)
            if dist < closest_dist:
                closest_dist = dist
                closest_bbox = bbox

        if closest_bbox is None:
            # No nearby secondary obstacles — steer left by default
            return vtheta + self.plan_turn_angle

        dx = closest_bbox.x - vx
        dy = closest_bbox.y - vy
        lateral = -dx * math.sin(vtheta) + dy * math.cos(vtheta)

        # Steer *away* from the secondary obstacle
        if lateral > 0:
            return vtheta - self.plan_turn_angle
        else:
            return vtheta + self.plan_turn_angle

    def _is_obstacle_clear(self, vx, vy, vtheta, bbox):
        """
        Return True once the virtual rover has moved far enough that the
        obstacle is no longer a threat — meaning it is either fully behind
        the rover OR displaced laterally beyond the safe corridor.

        This is the exit condition for avoidance mode and is deliberately
        stricter than _will_collide so the rover never re-enters the wobble
        zone while skirting an obstacle.
        """
        dx = bbox.x - vx
        dy = bbox.y - vy

        forward_dist = dx * math.cos(vtheta) + dy * math.sin(vtheta)
        lateral_dist = abs(-dx * math.sin(vtheta) + dy * math.cos(vtheta))

        # Clearance thresholds — wider than the detection thresholds so
        # the rover commits to the avoidance arc before resuming goal-seeking.
        clear_lateral = (
            self.rover_width / self.rover_safety_margin
            + bbox.width / 2.0
            + 0.3        # extra lateral clearance buffer
        )

        # Obstacle is clear when it is behind us OR fully off to the side
        behind = forward_dist <= 0
        beside = lateral_dist > clear_lateral

        return behind or beside

    def _step_virtual_rover(self, vx, vy, vtheta, avoiding_bbox):
        """
        Advance virtual rover one step.

        If avoiding_bbox is set the rover stays in avoidance mode for that
        specific obstacle regardless of whether the lookahead window still
        detects it, until _is_obstacle_clear() confirms it is safe to resume
        normal goal-seeking.

        Returns (new_vx, new_vy, new_vtheta, new_avoiding_bbox).
        """
        # --- Determine active obstacle ---
        if avoiding_bbox is not None:
            # Still avoiding a known obstacle — check whether we have cleared it
            if self._is_obstacle_clear(vx, vy, vtheta, avoiding_bbox):
                avoiding_bbox = None   # Cleared: resume normal steering

        if avoiding_bbox is None:
            # Check whether a new obstacle has entered the lookahead window
            avoiding_bbox = self._any_collision(vx, vy, vtheta)

        # --- Compute new heading ---
        if avoiding_bbox is not None:
            new_vtheta = self._avoidance_heading(vx, vy, vtheta, avoiding_bbox)
        else:
            # Normal target-seeking
            dx = self.target_x - vx
            dy = self.target_y - vy
            target_theta = math.atan2(dy, dx)
            heading_error = math.atan2(
                math.sin(target_theta - vtheta),
                math.cos(target_theta - vtheta)
            )
            new_vtheta = vtheta + 0.1 * heading_error

        new_vx = vx + self.forward_step * math.cos(new_vtheta)
        new_vy = vy + self.forward_step * math.sin(new_vtheta)
        return new_vx, new_vy, new_vtheta, avoiding_bbox

    # -----------------------------------------------------------------------
    # Trajectory planner — pure output, no rover state mutation
    # -----------------------------------------------------------------------

    def target_is_in_sight(self):
        """
        Return True if a straight, collision-free line exists from the rover's
        current position directly to the target — i.e. no obstacle safety
        corridor intersects the direct path.
        """
        return not self._segment_collides(
            self.rover_x, self.rover_y,
            self.target_x, self.target_y
        )

    def plan_trajectory(self):
        """
        Simulate a virtual rover forward from the current real pose and return
        a list of (x, y) waypoints that avoid all known obstacles while
        progressing toward the target.

        Avoidance mode is sticky: once the virtual rover starts avoiding an
        obstacle it holds that arc until _is_obstacle_clear() confirms the
        obstacle is fully behind or beside it.  This prevents the oscillating
        curve that occurs when an obstacle repeatedly enters and exits the
        constant-width lookahead window.

        The real rover's state (self.rover_x/y/theta) is never modified.

        Returns:
            Tuple of (waypoints, target_in_sight) where:
            - waypoints: pruned list of (x, y) tuples representing the planned path
            - target_in_sight: True if the target is directly reachable with
              no obstacles in the way
        """
        target_in_sight = self.target_is_in_sight()

        vx, vy, vtheta = self.rover_x, self.rover_y, self.rover_theta
        avoiding_bbox = None
        waypoints = [(vx, vy)]

        for _ in range(self.max_plan_steps):
            dx = self.target_x - vx
            dy = self.target_y - vy
            if (dx ** 2 + dy ** 2) < self.target_threshold ** 2:
                waypoints.append((self.target_x, self.target_y))
                break

            vx, vy, vtheta, avoiding_bbox = self._step_virtual_rover(
                vx, vy, vtheta, avoiding_bbox
            )
            waypoints.append((vx, vy))

        return self._shortcut_trajectory(waypoints), target_in_sight

    def _segment_collides(self, x0, y0, x1, y1):
        """
        Return True if the straight line segment from (x0,y0) to (x1,y1)
        passes through any obstacle's safety corridor.

        The segment is sampled at steps of forward_step so the resolution
        matches the planner and no gap can swallow an obstacle.
        """
        seg_len = math.hypot(x1 - x0, y1 - y0)
        if seg_len < 1e-6:
            return False

        steps = max(2, int(seg_len / self.forward_step) + 1)
        heading = math.atan2(y1 - y0, x1 - x0)

        for i in range(steps + 1):
            t = i / steps
            sx = x0 + t * (x1 - x0)
            sy = y0 + t * (y1 - y0)
            if self._any_collision(sx, sy, heading):
                return True
        return False

    def _shortcut_trajectory(self, waypoints):
        """
        Greedy path shortcutting: walk the waypoint list and skip any
        intermediate point that can be connected to a later point by a
        straight, collision-free segment.

        This removes the stutter/oscillation artefacts that arise when the
        planner makes many tiny heading corrections near obstacle edges,
        producing a clean path with only the geometrically necessary turns.

        Parameters:
            waypoints: raw list of (x, y) tuples from the planner

        Returns:
            Pruned list of (x, y) tuples.
        """
        if len(waypoints) < 3:
            return waypoints

        pruned = [waypoints[0]]
        i = 0

        while i < len(waypoints) - 1:
            # Try to connect pruned[-1] directly to the furthest possible
            # point without hitting any obstacle.
            best_j = i + 1
            for j in range(len(waypoints) - 1, i + 1, -1):
                x0, y0 = pruned[-1]
                x1, y1 = waypoints[j]
                if not self._segment_collides(x0, y0, x1, y1):
                    best_j = j
                    break
            pruned.append(waypoints[best_j])
            i = best_j

        return pruned

    def create_trajectory_marker(self, waypoints, target_in_sight):
        """
        Build a LINE_STRIP marker that visualises the planned trajectory.

        Color encodes whether the target is directly reachable:
          - Green  (0, 1, 0): target is in sight — no obstacles on direct path
          - Yellow (1, 1, 0): target is obscured — avoidance path is active

        Parameters:
            waypoints: list of (x, y) tuples from plan_trajectory()
            target_in_sight: bool returned by plan_trajectory()

        Returns:
            Marker message (LINE_STRIP)
        """
        marker = Marker()
        marker.header.frame_id = "laser"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "planned_trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        for wx, wy in waypoints:
            p = Point()
            p.x = wx
            p.y = wy
            p.z = 0.05          # Slightly above ground so it is visible
            marker.points.append(p)

        marker.scale.x = 0.05   # Line width

        if target_in_sight:
            # Green — clear line of sight to target
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            # Yellow — avoidance path active
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0

        marker.color.a = 1.0

        return marker
        
    def publish_markers(self):
        """
        Main publishing function.

        Computes a local avoidance trajectory and publishes it together with
        the static rover position and obstacle bounding boxes.
        The real rover pose is never modified here.
        """
        # --- Plan trajectory (read-only on rover state) ---
        waypoints, target_in_sight = self.plan_trajectory()

        self.get_logger().info(
            f'Target in sight: {target_in_sight}',
            throttle_duration_sec=10.0
        )

        # --- Publish planned trajectory ---
        traj_marker = self.create_trajectory_marker(waypoints, target_in_sight)
        self.trajectory_publisher.publish(traj_marker)

        # --- Publish rover position (static in production) ---
        rover_marker = self.create_rover_marker()
        self.rover_marker_publisher.publish(rover_marker)
        
        # --- Publish bounding boxes ---
        marker_array = MarkerArray()
        for i, bbox in enumerate(self.bounding_boxes):
            marker = self.create_bounding_box_marker(marker_id=i, bbox=bbox)
            marker_array.markers.append(marker)
        self.marker_publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = RoutingSimulation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()