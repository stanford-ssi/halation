"""
mock_vision.py  —  Mock GroundingDINO obstacle publisher
=========================================================
Simulates what grounding_detect_node.py would publish — a stream of
obstacle detections on /obstacle_detections.

Runs three scenarios in sequence so you can watch rover_nav react:

  Scenario 0  (0–10 s)   : No obstacles — rover drives straight to goal
  Scenario 1  (10–25 s)  : One obstacle dead ahead at 4 m — rover avoids it
  Scenario 2  (25–40 s)  : Obstacle sweeps left→right — rover tracks avoidance
  Scenario 3  (40 s+)    : Two obstacles, one left one right — rover squeezes through

Published Topics:
  /obstacle_detections  (std_msgs/String)
      JSON: {"obstacles": [{"angle_rad", "distance_m", "confidence", "width_m", "class_name"}]}

Run standalone:
    ros2 run avoidance_rerouting mock_vision

Or use the mock_test.launch.py which runs everything together.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

PUBLISH_HZ = 10.0   # matches real grounding_detect_node rate


class MockVisionNode(Node):

    def __init__(self):
        super().__init__("mock_vision")

        self.pub = self.create_publisher(String, "/obstacle_detections", 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._timer_cb)

        self._start_time = time.monotonic()
        self.get_logger().info("Mock vision node started.")
        self.get_logger().info(
            "Scenario 0 (0-10s):  No obstacles\n"
            "Scenario 1 (10-25s): Obstacle dead ahead at 4m\n"
            "Scenario 2 (25-40s): Obstacle sweeping left→right\n"
            "Scenario 3 (40s+):   Two obstacles, one left one right"
        )

    def _timer_cb(self):
        elapsed = time.monotonic() - self._start_time
        obstacles = self._get_obstacles(elapsed)
        msg = String()
        msg.data = json.dumps({"obstacles": obstacles})
        self.pub.publish(msg)

        if obstacles:
            self.get_logger().info(
                f"t={elapsed:.1f}s  publishing {len(obstacles)} obstacle(s): " +
                ", ".join(
                    f"{o['class_name']} "
                    f"dist={o['distance_m']:.1f}m "
                    f"angle={math.degrees(o['angle_rad']):.0f}deg"
                    for o in obstacles
                ),
                throttle_duration_sec=2.0,
            )

    def _get_obstacles(self, elapsed: float) -> list:
        """Return a list of obstacle dicts for the current scenario."""

        # ── Scenario 0: clear path ────────────────────────────────────────────
        if elapsed < 10.0:
            return []

        # ── Scenario 1: one obstacle dead ahead ──────────────────────────────
        elif elapsed < 25.0:
            return [self._obs(
                class_name  = "rock",
                angle_deg   = 0.0,       # dead ahead
                distance_m  = 4.0,
                confidence  = 0.85,
                width_m     = 1.0,
            )]

        # ── Scenario 2: obstacle sweeps left→right ────────────────────────────
        elif elapsed < 40.0:
            # oscillates between -30° and +30° over 5 seconds
            t = elapsed - 25.0
            sweep_angle_deg = 30.0 * math.sin(2 * math.pi * t / 5.0)
            return [self._obs(
                class_name  = "person",
                angle_deg   = sweep_angle_deg,
                distance_m  = 3.5,
                confidence  = 0.90,
                width_m     = 0.5,
            )]

        # ── Scenario 3: two obstacles, gap between them ───────────────────────
        else:
            return [
                self._obs(
                    class_name = "rock",
                    angle_deg  = -20.0,   # to the left
                    distance_m = 3.0,
                    confidence = 0.80,
                    width_m    = 0.8,
                ),
                self._obs(
                    class_name = "boulder",
                    angle_deg  = +20.0,   # to the right
                    distance_m = 3.0,
                    confidence = 0.80,
                    width_m    = 0.8,
                ),
            ]

    @staticmethod
    def _obs(class_name: str, angle_deg: float, distance_m: float,
             confidence: float, width_m: float) -> dict:
        return {
            "class_name":  class_name,
            "angle_rad":   round(math.radians(angle_deg), 4),
            "distance_m":  round(distance_m, 2),
            "confidence":  round(confidence, 2),
            "width_m":     round(width_m, 2),
        }


def main(args=None):
    rclpy.init(args=args)
    node = MockVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()