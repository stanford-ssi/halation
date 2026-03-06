"""
mock_gps_imu.py  —  Mock GPS + IMU publisher for rover_nav testing
===================================================================
Simulates a rover starting at a fixed GPS position and driving
straight toward the goal, so you can test rover_nav.py without
real hardware.

What it publishes:
  /gps/fix      (sensor_msgs/NavSatFix)  — position updates at 5 Hz
  /imu_heading  (std_msgs/String)        — heading JSON at 20 Hz

The simulated rover moves in a straight line toward a fixed target
at MOCK_SPEED m/s. Heading is always pointed at the goal.

Run:
    ros2 run avoidance_rerouting mock_gps_imu

Then in another terminal launch the nav stack:
    ros2 launch avoidance_rerouting navigation.launch.py \
        goal_lat:=37.42850 goal_lon:=-122.16900
"""

import json
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String

# ── Starting position ─────────────────────────────────────────────────────────
# Change these to wherever you want the simulated rover to start
START_LAT =  37.42750
START_LON = -122.16970

# ── Simulated movement ────────────────────────────────────────────────────────
MOCK_SPEED_MPS = 1.0      # metres per second
GPS_HZ         = 5.0      # how often GPS publishes
IMU_HZ         = 20.0     # how often IMU publishes

# ── Earth radius for position stepping ───────────────────────────────────────
EARTH_RADIUS_M = 6371000.0


class MockGpsImuNode(Node):

    def __init__(self):
        super().__init__("mock_gps_imu")

        # Current simulated position
        self.lat = START_LAT
        self.lon = START_LON
        self.heading_rad = 0.0   # ENU: 0 = East

        # Publishers
        self.gps_pub     = self.create_publisher(NavSatFix, "/gps/fix",     10)
        self.heading_pub = self.create_publisher(String,    "/imu_heading", 10)

        # Timers
        self.create_timer(1.0 / GPS_HZ, self._publish_gps)
        self.create_timer(1.0 / IMU_HZ, self._publish_imu)

        self.get_logger().info(
            f"Mock GPS/IMU started at ({START_LAT:.7f}, {START_LON:.7f})"
        )
        self.get_logger().info(
            "Rover is stationary — heading East (0 rad). "
            "Publish /mock_velocity to move it, or it stays put for clean testing."
        )

        # Optional: subscribe to velocity commands from rover_nav motor output
        # so the simulated position actually moves when motor_vector is published
        self.create_subscription(
            String, "/motor_vector", self._motor_cb, 10
        )
        self._last_x = 0.0
        self._last_y = 0.0

    # ── Move simulation based on motor commands ───────────────────────────────

    def _motor_cb(self, msg: String):
        """
        Read motor_vector and move the simulated GPS position.
        This closes the loop: rover_nav → motor → simulated movement → GPS update.
        """
        try:
            outer = json.loads(msg.data)
            inner = json.loads(outer.get("data", "{}"))
            x = float(inner.get("x", 0.0))   # steering
            y = float(inner.get("y", 0.0))   # speed
        except (json.JSONDecodeError, ValueError):
            return

        # Update heading based on steering command
        # x steers left/right, changing heading over time
        self.heading_rad += x * 0.1   # scale: full steer = ~0.1 rad/step
        self.heading_rad = (self.heading_rad + math.pi) % (2 * math.pi) - math.pi

        # Move forward in current heading direction
        dt        = 1.0 / IMU_HZ
        speed_mps = y * MOCK_SPEED_MPS
        dist_m    = speed_mps * dt

        east_m  = dist_m * math.cos(self.heading_rad)
        north_m = dist_m * math.sin(self.heading_rad)

        # Convert ENU movement to GPS delta
        self.lat += math.degrees(north_m / EARTH_RADIUS_M)
        self.lon += math.degrees(
            east_m / (EARTH_RADIUS_M * math.cos(math.radians(self.lat)))
        )

    # ── Publishers ────────────────────────────────────────────────────────────

    def _publish_gps(self):
        msg = NavSatFix()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps"
        msg.status.status   = NavSatStatus.STATUS_FIX
        msg.status.service  = NavSatStatus.SERVICE_GPS
        msg.latitude        = self.lat
        msg.longitude       = self.lon
        msg.altitude        = 10.0
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        msg.position_covariance      = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.gps_pub.publish(msg)

    def _publish_imu(self):
        msg      = String()
        msg.data = json.dumps({
            "heading_rad": round(self.heading_rad, 4),
            "accuracy_rad": 0.05,
        })
        self.heading_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockGpsImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()