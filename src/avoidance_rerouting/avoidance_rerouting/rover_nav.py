"""
rover_nav.py  —  SSI Rover Navigation Node (GPS + Vision, 400 m prototype)
==========================================================================
Single node that does everything:
  1. Reads GPS  → rover position in ENU metres
  2. Reads IMU  → rover heading
  3. Reads vision detections → obstacle bounding boxes
  4. Runs virtual path simulation (from routing_simulation.py) to plan around obstacles
  5. Steers toward planned path → publishes /motor_vector → motor_control.py moves wheels

Goal is set via launch file parameters goal_lat / goal_lon.
Can also be overridden at runtime via /goal_gps topic.

Subscribed Topics:
  /gps/fix              (sensor_msgs/NavSatFix)   rover GPS position
  /imu_heading          (std_msgs/String)          JSON {"heading_rad": float}
  /obstacle_detections  (std_msgs/String)          JSON {"obstacles": [...]}
  /goal_gps             (std_msgs/String)          JSON {"lat": float, "lon": float}

Published Topics:
  /motor_vector         (std_msgs/String)          JSON (double-encoded for rosbridge)
  /nav_status           (std_msgs/String)          JSON status for Foxglove dashboard
  /planned_trajectory   (visualization_msgs/Marker)  RViz path line
  /rover_position       (visualization_msgs/Marker)  RViz rover cube
  /object_bounding_boxes (visualization_msgs/MarkerArray) RViz obstacles
"""

import json
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# ═══════════════════════════════════════════════════════════════════════════════
#  Constants
# ═══════════════════════════════════════════════════════════════════════════════

EARTH_RADIUS_M = 6371000.0

# ── Navigation tuning ─────────────────────────────────────────────────────────
HEADING_KP          = 0.7    # proportional gain: heading_error → steering
BASE_SPEED          = 0.45   # forward speed (0–1 scale)
MIN_SPEED           = 0.15   # minimum speed when correcting large heading error
GOAL_ARRIVAL_M      = 0.8    # stop when within this distance of goal (metres)
WAYPOINT_LOOKAHEAD  = 3      # index into planned path to steer toward

# ── Path planner tuning ───────────────────────────────────────────────────────
FORWARD_STEP        = 0.15   # virtual rover step size (metres)
LOOKAHEAD_DISTANCE  = 3.5    # collision check horizon (metres)
ROVER_SAFETY_MARGIN = 0.5    # (rover_dim / margin) + (obs_dim / 2) = safe corridor
MAX_PLAN_STEPS      = 400    # max virtual steps per plan cycle
PLAN_TURN_ANGLE_DEG = 5.0    # degrees to turn per avoidance step
TARGET_THRESHOLD    = 0.5    # virtual rover arrival radius (metres)

# ── Vision / obstacle tuning ─────────────────────────────────────────────────
MIN_CONFIDENCE      = 0.4    # ignore detections below this
MAX_OBS_DISTANCE_M  = 8.0    # ignore detections beyond this
MIN_OBS_DISTANCE_M  = 0.3    # ignore detections closer than this (noise floor)
DEFAULT_OBS_WIDTH_M = 0.5    # assumed width if vision cannot estimate

# ── Rover physical dimensions ─────────────────────────────────────────────────
ROVER_WIDTH_M       = 0.6
ROVER_LENGTH_M      = 0.8

# ── Control loop rate ─────────────────────────────────────────────────────────
CONTROL_HZ          = 5.0    # Hz — how often to plan + publish motor command


# ═══════════════════════════════════════════════════════════════════════════════
#  Coordinate helpers
# ═══════════════════════════════════════════════════════════════════════════════

def gps_to_enu(lat: float, lon: float,
               origin_lat: float, origin_lon: float) -> tuple:
    """Convert GPS (degrees) to local ENU (metres) relative to origin."""
    north = math.radians(lat - origin_lat) * EARTH_RADIUS_M
    east  = (math.radians(lon - origin_lon) * EARTH_RADIUS_M
             * math.cos(math.radians(origin_lat)))
    return east, north


def normalize_angle(angle: float) -> float:
    """Normalise angle to [-π, π]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


# ═══════════════════════════════════════════════════════════════════════════════
#  BoundingBox  (unchanged from routing_simulation.py)
# ═══════════════════════════════════════════════════════════════════════════════

class BoundingBox:
    """Obstacle represented as a box in global ENU coordinates."""

    def __init__(self, height: float, width: float,
                 distance: float, angle_deg: float,
                 rover_pos: tuple):
        """
        Parameters
        ----------
        height     : vertical size (m) — used for RViz only
        width      : horizontal size (m)
        distance   : metres from rover centre to obstacle centre
        angle_deg  : angle from rover's forward axis (degrees)
                     negative = obstacle to LEFT, positive = to RIGHT
        rover_pos  : (rover_x, rover_y, rover_theta) at time of detection
        """
        rover_x, rover_y, rover_theta = rover_pos

        self.height = height
        self.width  = width
        self.length = width   # assume square footprint

        dist_angle_rad = math.radians(angle_deg) + rover_theta
        self.x     = rover_x + distance * math.cos(dist_angle_rad)
        self.y     = rover_y + distance * math.sin(dist_angle_rad)
        self.angle = dist_angle_rad


# ═══════════════════════════════════════════════════════════════════════════════
#  RoverNav Node
# ═══════════════════════════════════════════════════════════════════════════════

class RoverNavNode(Node):

    def __init__(self):
        super().__init__("rover_nav")

        # ── Declare launch parameters ─────────────────────────────────────────
        self.declare_parameter("goal_lat", 0.0)
        self.declare_parameter("goal_lon", 0.0)

        # ── State  (all writes go through _state_lock) ────────────────────────
        self._state_lock    = threading.Lock()

        self.origin_gps: tuple | None = None   # (lat, lon) set on first GPS fix
        self.rover_x    = 0.0                  # ENU east   (metres)
        self.rover_y    = 0.0                  # ENU north  (metres)
        self.rover_theta = 0.0                 # heading radians (ENU: 0=East CCW+)

        self.target_x: float | None = None     # ENU goal east
        self.target_y: float | None = None     # ENU goal north
        self.goal_lat: float | None = None
        self.goal_lon: float | None = None

        self.bounding_boxes: list = []
        self.have_fix       = False
        self.state          = "WAITING_FOR_FIX"   # state machine

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            NavSatFix, "/gps/fix", self._gps_cb, 10)
        self.create_subscription(
            String, "/imu_heading", self._heading_cb, 10)
        self.create_subscription(
            String, "/obstacle_detections", self._obs_cb, 10)
        self.create_subscription(
            String, "/goal_gps", self._goal_topic_cb, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        self.motor_pub  = self.create_publisher(String,     "/motor_vector",          10)
        self.status_pub = self.create_publisher(String,     "/nav_status",            10)
        self.traj_pub   = self.create_publisher(Marker,     "/planned_trajectory",    10)
        self.rover_pub  = self.create_publisher(Marker,     "/rover_position",        10)
        self.bbox_pub   = self.create_publisher(MarkerArray,"/object_bounding_boxes", 10)

        # ── Control loop ──────────────────────────────────────────────────────
        self.create_timer(1.0 / CONTROL_HZ, self._control_loop)

        self.get_logger().info("RoverNav node started.")
        self.get_logger().info(
            "Waiting for first GPS fix before accepting goal..."
        )

        # Toggle: Control Mode:
        self.nav_mode = "auto"
        self.create_subscription(String, "/nav_mode", self._mode_cb, 10)

    # ═══════════════════════════════════════════════════════════════════════════
    #  Subscriber callbacks
    # ══════════════════════════════════════════════════════════════════════════                                                                
    def _gps_cb(self, msg: NavSatFix):
        """Update ENU position.  Set origin and goal on first fix."""
        with self._state_lock:
            if self.origin_gps is None:
                self.origin_gps = (msg.latitude, msg.longitude)
                self.get_logger().info(
                    f"ENU origin set: {msg.latitude:.7f}, {msg.longitude:.7f}"
                )
                # Now that origin is known, resolve launch-param goal
                self._resolve_goal_params()

            east, north = gps_to_enu(
                msg.latitude, msg.longitude,
                self.origin_gps[0], self.origin_gps[1],
            )
            self.rover_x  = east
            self.rover_y  = north
            self.have_fix = True

            if self.state == "WAITING_FOR_FIX":
                self.state = "WAITING_FOR_GOAL" if self.target_x is None else "NAVIGATING"

    def _mode_cb(self, msg: String):
        mode = msg.data.strip()
        if mode in ("auto", "manual"):
            self.nav_mode = mode
            self.get_logger().info(f"Mode: {mode}")

    def _heading_cb(self, msg: String):
        """
        Update rover heading.
        Expected JSON: {"heading_rad": float}
        ENU convention — 0 = East, counterclockwise positive.
        If your IMU driver outputs compass bearing, it must convert:
            heading_rad = radians(90 - compass_bearing_deg)
        """
        try:
            data = json.loads(msg.data)
            with self._state_lock:
                self.rover_theta = float(data["heading_rad"])
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.get_logger().warn(
                f"Bad /imu_heading message: {e}", throttle_duration_sec=5.0
            )

    def _obs_cb(self, msg: String):
        """
        Rebuild bounding_boxes from latest vision detections.
        Expected JSON:
          {"obstacles": [{"angle_rad": float, "distance_m": float,
                          "confidence": float, "width_m": float|null}]}
        """
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        with self._state_lock:
            rx, ry, rt = self.rover_x, self.rover_y, self.rover_theta

        new_boxes = []
        for obs in data.get("obstacles", []):
            conf = float(obs.get("confidence", 0.0))
            dist = obs.get("distance_m")
            ang  = obs.get("angle_rad")

            if conf < MIN_CONFIDENCE:
                continue
            if dist is None or float(dist) < MIN_OBS_DISTANCE_M:
                continue
            if float(dist) > MAX_OBS_DISTANCE_M:
                continue
            if ang is None:
                continue

            width = float(obs.get("width_m") or DEFAULT_OBS_WIDTH_M)
            new_boxes.append(BoundingBox(
                height    = 0.6,
                width     = width,
                distance  = float(dist),
                angle_deg = math.degrees(float(ang)),
                rover_pos = (rx, ry, rt),
            ))

        with self._state_lock:
            self.bounding_boxes = new_boxes

    def _goal_topic_cb(self, msg: String):
        """
        Accept a goal override from /goal_gps at runtime.
        JSON: {"lat": float, "lon": float}
        """
        try:
            outer = json.loads(msg.data)
            # Unwrap rosbridge double-encoding if present
            inner = json.loads(outer["data"]) if "data" in outer else outer
            lat = float(inner["lat"])
            lon = float(inner["lon"])
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.get_logger().error(f"Bad /goal_gps message: {e}")
            return

        with self._state_lock:
            if self.origin_gps is None:
                self.get_logger().warn("No GPS fix yet — cannot accept goal.")
                return
            self._set_goal(lat, lon)

    # ═══════════════════════════════════════════════════════════════════════════
    #  Goal helpers
    # ═══════════════════════════════════════════════════════════════════════════

    def _resolve_goal_params(self):
        """
        Called once origin is known.
        Reads goal_lat / goal_lon launch parameters and sets goal if valid.
        Must be called with _state_lock held.
        """
        lat = self.get_parameter("goal_lat").get_parameter_value().double_value
        lon = self.get_parameter("goal_lon").get_parameter_value().double_value
        if lat == 0.0 and lon == 0.0:
            self.get_logger().warn(
                "No goal set via launch params. "
                "Send goal: ros2 topic pub /goal_gps std_msgs/String "
                "'{\"data\": \"{\\\"lat\\\": XX.XXX, \\\"lon\\\": YY.YYY}\"}'"
            )
            return
        self._set_goal(lat, lon)

    def _set_goal(self, lat: float, lon: float):
        """
        Convert goal GPS to ENU and transition to NAVIGATING.
        Must be called with _state_lock held.
        """
        east, north = gps_to_enu(
            lat, lon,
            self.origin_gps[0], self.origin_gps[1],
        )
        self.target_x  = east
        self.target_y  = north
        self.goal_lat  = lat
        self.goal_lon  = lon
        self.state     = "NAVIGATING" if self.have_fix else "WAITING_FOR_FIX"
        self.get_logger().info(
            f"Goal set → lat={lat:.7f} lon={lon:.7f} "
            f"ENU=({east:.1f} m E, {north:.1f} m N)  "
            f"dist={math.hypot(east - self.rover_x, north - self.rover_y):.1f} m"
        )

    # ═══════════════════════════════════════════════════════════════════════════
    #  Path planner  (ported directly from routing_simulation.py)
    # ═══════════════════════════════════════════════════════════════════════════

    def _will_collide(self, vx: float, vy: float, vtheta: float,
                      bbox: BoundingBox) -> bool:
        dx  = bbox.x - vx
        dy  = bbox.y - vy
        fwd = dx * math.cos(vtheta) + dy * math.sin(vtheta)
        lat = -dx * math.sin(vtheta) + dy * math.cos(vtheta)

        safe_fwd = (LOOKAHEAD_DISTANCE
                    + ROVER_LENGTH_M / ROVER_SAFETY_MARGIN
                    + bbox.length / 2.0)
        safe_lat = (ROVER_WIDTH_M / ROVER_SAFETY_MARGIN
                    + bbox.width / 2.0)

        return (0 < fwd < safe_fwd) and (abs(lat) < safe_lat)

    def _any_collision(self, vx: float, vy: float,
                       vtheta: float) -> BoundingBox | None:
        for bbox in self.bounding_boxes:
            if self._will_collide(vx, vy, vtheta, bbox):
                return bbox
        return None

    def _avoidance_heading(self, vx: float, vy: float, vtheta: float,
                       blocking: BoundingBox) -> float:
        dx = blocking.x - vx
        dy = blocking.y - vy

        clear_half = (ROVER_WIDTH_M / ROVER_SAFETY_MARGIN
                    + blocking.width / 2.0 + 0.3)

        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return vtheta + math.radians(PLAN_TURN_ANGLE_DEG)

        angle_to_obstacle = math.atan2(dy, dx)
        angular_half = math.asin(min(clear_half / dist, 1.0))

        heading_pass_right = angle_to_obstacle - angular_half
        heading_pass_left  = angle_to_obstacle + angular_half

        def herr(target):
            return abs(math.atan2(math.sin(target - vtheta),
                                math.cos(target - vtheta)))

        turn = math.radians(PLAN_TURN_ANGLE_DEG)
        if herr(heading_pass_right) < herr(heading_pass_left):
            return vtheta - turn   # pass to the right of the obstacle
        else:
            return vtheta + turn   # pass to the left of the obstacle

    def _obstacle_clear(self, vx: float, vy: float, vtheta: float,
                        bbox: BoundingBox) -> bool:
        dx  = bbox.x - vx
        dy  = bbox.y - vy
        fwd = dx * math.cos(vtheta) + dy * math.sin(vtheta)
        lat = abs(-dx * math.sin(vtheta) + dy * math.cos(vtheta))
        clear_lat = (ROVER_WIDTH_M / ROVER_SAFETY_MARGIN
                     + bbox.width / 2.0 + 0.3)
        return fwd <= 0 or lat > clear_lat

    def _step_virtual(self, vx, vy, vtheta, avoiding):
        """Advance virtual rover one step. Returns (vx, vy, vtheta, avoiding)."""
        if avoiding is not None and self._obstacle_clear(vx, vy, vtheta, avoiding):
            avoiding = None

        if avoiding is None:
            avoiding = self._any_collision(vx, vy, vtheta)

        if avoiding is not None:
            new_theta = self._avoidance_heading(vx, vy, vtheta, avoiding)
        else:
            dx  = self.target_x - vx
            dy  = self.target_y - vy
            tgt = math.atan2(dy, dx)
            err = math.atan2(math.sin(tgt - vtheta), math.cos(tgt - vtheta))
            new_theta = vtheta + 0.1 * err

        new_vx = vx + FORWARD_STEP * math.cos(new_theta)
        new_vy = vy + FORWARD_STEP * math.sin(new_theta)
        return new_vx, new_vy, new_theta, avoiding

    def _segment_collides(self, x0, y0, x1, y1) -> bool:
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg < 1e-6:
            return False
        steps   = max(2, int(seg / FORWARD_STEP) + 1)
        heading = math.atan2(y1 - y0, x1 - x0)
        for i in range(steps + 1):
            t = i / steps
            if self._any_collision(x0 + t*(x1-x0), y0 + t*(y1-y0), heading):
                return True
        return False

    def _shortcut(self, wps: list) -> list:
        if len(wps) < 3:
            return wps
        pruned = [wps[0]]
        i = 0
        while i < len(wps) - 1:
            best = i + 1
            for j in range(len(wps) - 1, i + 1, -1):
                if not self._segment_collides(*pruned[-1], *wps[j]):
                    best = j
                    break
            pruned.append(wps[best])
            i = best
        return pruned

    def _plan(self) -> tuple:
        """
        Returns (waypoints, target_in_sight).
        waypoints: list of (east, north) ENU metres from current pos to goal.
        """
        in_sight = not self._segment_collides(
            self.rover_x, self.rover_y,
            self.target_x, self.target_y,
        )

        vx, vy, vt = self.rover_x, self.rover_y, self.rover_theta
        avoiding   = None
        wps        = [(vx, vy)]

        for _ in range(MAX_PLAN_STEPS):
            if (self.target_x - vx)**2 + (self.target_y - vy)**2 < TARGET_THRESHOLD**2:
                wps.append((self.target_x, self.target_y))
                break
            vx, vy, vt, avoiding = self._step_virtual(vx, vy, vt, avoiding)
            wps.append((vx, vy))

        return self._shortcut(wps), in_sight

    # ═══════════════════════════════════════════════════════════════════════════
    #  Motor output
    # ═══════════════════════════════════════════════════════════════════════════

    def _motor_from_path(self, waypoints: list) -> tuple:
        """
        Given planned waypoints, compute (x_cmd, y_cmd).
        Steers toward waypoints[WAYPOINT_LOOKAHEAD].
        Returns (0, 0) if at goal.
        """
        if len(waypoints) < 2:
            return 0.0, 0.0

        idx    = min(WAYPOINT_LOOKAHEAD, len(waypoints) - 1)
        target = waypoints[idx]

        dx   = target[0] - self.rover_x
        dy   = target[1] - self.rover_y
        dist = math.hypot(dx, dy)

        if dist < GOAL_ARRIVAL_M:
            return 0.0, 0.0

        desired  = math.atan2(dy, dx)
        err      = normalize_angle(desired - self.rover_theta)

        # Steering — proportional to heading error
        x_cmd = HEADING_KP * err
        x_cmd = max(-1.0, min(1.0, x_cmd))

        # Speed — slow down when turning hard
        speed_scale = max(MIN_SPEED / BASE_SPEED,
                          1.0 - abs(err) / math.pi)
        y_cmd = BASE_SPEED * speed_scale

        return x_cmd, y_cmd

    def _publish_motor(self, x: float, y: float):
        """
        Publish motor vector in the double-encoded JSON format
        that motor_control.py already expects.
        """
        inner = json.dumps({"x": round(x, 3), "y": round(y, 3)})
        outer = json.dumps({"data": inner})
        msg   = String()
        msg.data = outer
        self.motor_pub.publish(msg)

    # ═══════════════════════════════════════════════════════════════════════════
    #  Main control loop  (runs at CONTROL_HZ)
    # ═══════════════════════════════════════════════════════════════════════════

    def _control_loop(self):
        if self.nav_mode == "manual":
            return

        with self._state_lock:
            state = self.state

        # ── Not ready yet ─────────────────────────────────────────────────────
        if state == "WAITING_FOR_FIX":
            self.get_logger().info(
                "Waiting for GPS fix...", throttle_duration_sec=5.0
            )
            return

        if state == "WAITING_FOR_GOAL":
            self.get_logger().info(
                "GPS fix acquired. Waiting for goal...",
                throttle_duration_sec=5.0,
            )
            return

        # ── Arrived ───────────────────────────────────────────────────────────
        if state == "ARRIVED":
            self._publish_motor(0.0, 0.0)
            return

        # ── NAVIGATING ────────────────────────────────────────────────────────
        # Check arrival
        dist = math.hypot(
            self.target_x - self.rover_x,
            self.target_y - self.rover_y,
        )
        if dist < GOAL_ARRIVAL_M:
            with self._state_lock:
                self.state = "ARRIVED"
            self._publish_motor(0.0, 0.0)
            self.get_logger().info(
                f"GOAL REACHED! Stopped at ({self.rover_x:.1f}, {self.rover_y:.1f}) m"
            )
            self._publish_status("ARRIVED", [], False)
            return

        # Plan path
        waypoints, in_sight = self._plan()

        # Compute and send motor command
        x_cmd, y_cmd = self._motor_from_path(waypoints)
        self._publish_motor(x_cmd, y_cmd)

        # Log at 1 Hz
        self.get_logger().info(
            f"dist={dist:.1f}m  "
            f"obs={len(self.bounding_boxes)}  "
            f"in_sight={in_sight}  "
            f"x={x_cmd:.2f}  y={y_cmd:.2f}",
            throttle_duration_sec=1.0,
        )

        # Status + visualisation
        self._publish_status(state, waypoints, in_sight)
        self._publish_viz(waypoints, in_sight)

    # ═══════════════════════════════════════════════════════════════════════════
    #  Status + RViz publishers
    # ═══════════════════════════════════════════════════════════════════════════

    def _publish_status(self, state: str, waypoints: list, in_sight: bool):
        dist = (math.hypot(self.target_x - self.rover_x,
                           self.target_y - self.rover_y)
                if self.target_x is not None else None)
        msg       = String()
        msg.data  = json.dumps({
            "state":           state,
            "rover_x":         round(self.rover_x, 2),
            "rover_y":         round(self.rover_y, 2),
            "heading_deg":     round(math.degrees(self.rover_theta), 1),
            "target_x":        round(self.target_x, 2) if self.target_x else None,
            "target_y":        round(self.target_y, 2) if self.target_y else None,
            "distance_to_goal_m": round(dist, 1) if dist else None,
            "num_obstacles":   len(self.bounding_boxes),
            "target_in_sight": in_sight,
            "num_waypoints":   len(waypoints),
        })
        self.status_pub.publish(msg)

    def _publish_viz(self, waypoints: list, in_sight: bool):
        now = self.get_clock().now().to_msg()

        # ── Trajectory line ───────────────────────────────────────────────────
        tm             = Marker()
        tm.header.frame_id = "map"
        tm.header.stamp    = now
        tm.ns, tm.id   = "trajectory", 0
        tm.type        = Marker.LINE_STRIP
        tm.action      = Marker.ADD
        tm.scale.x     = 0.05
        tm.color.a     = 1.0
        if in_sight:
            tm.color.r, tm.color.g, tm.color.b = 0.0, 1.0, 0.0   # green
        else:
            tm.color.r, tm.color.g, tm.color.b = 1.0, 1.0, 0.0   # yellow
        for wx, wy in waypoints:
            p = Point(); p.x = wx; p.y = wy; p.z = 0.05
            tm.points.append(p)
        self.traj_pub.publish(tm)

        # ── Rover cube ────────────────────────────────────────────────────────
        rm             = Marker()
        rm.header.frame_id = "map"
        rm.header.stamp    = now
        rm.ns, rm.id   = "rover", 0
        rm.type        = Marker.CUBE
        rm.action      = Marker.ADD
        rm.pose.position.x = self.rover_x
        rm.pose.position.y = self.rover_y
        rm.pose.position.z = 0.25
        rm.scale.x = ROVER_LENGTH_M
        rm.scale.y = ROVER_WIDTH_M
        rm.scale.z = 0.5
        rm.color.r, rm.color.g, rm.color.b, rm.color.a = 0.0, 0.4, 1.0, 0.9
        self.rover_pub.publish(rm)

        # ── Obstacle boxes ────────────────────────────────────────────────────
        ma = MarkerArray()
        for i, bbox in enumerate(self.bounding_boxes):
            bm             = Marker()
            bm.header.frame_id = "map"
            bm.header.stamp    = now
            bm.ns, bm.id   = "obstacles", i
            bm.type        = Marker.CUBE
            bm.action      = Marker.ADD
            bm.pose.position.x = bbox.x
            bm.pose.position.y = bbox.y
            bm.pose.position.z = bbox.height / 2.0
            bm.scale.x = bbox.length
            bm.scale.y = bbox.width
            bm.scale.z = bbox.height
            bm.color.r, bm.color.g, bm.color.b, bm.color.a = 1.0, 0.3, 0.0, 0.75
            ma.markers.append(bm)
        self.bbox_pub.publish(ma)


# ═══════════════════════════════════════════════════════════════════════════════
#  Entry point
# ═══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = RoverNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()