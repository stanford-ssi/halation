#!/usr/bin/env python3
"""
Point Cloud Accumulator Node

Subscribes to 2D lidar scans and servo pitch angle, rotates scan points
into 3D space based on pitch, accumulates points, and periodically saves
to a PLY point cloud file.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import math
import os
from datetime import datetime
from collections import deque
import threading


# How often to save point cloud (seconds)
SAVE_INTERVAL = 10.0

# Maximum points to keep in memory (prevent OOM)
MAX_POINTS = 500000

# Output directory for point clouds
OUTPUT_DIR = os.path.expanduser('~/halation/pointclouds')


class PointCloudAccumulator(Node):
    def __init__(self):
        super().__init__('pointcloud_accumulator')

        # Ensure output directory exists
        os.makedirs(OUTPUT_DIR, exist_ok=True)

        # Storage for accumulated 3D points (x, y, z)
        self.points = []
        self.points_lock = threading.Lock()

        # Latest pitch angle with timestamp
        self.latest_pitch = 0.0
        self.pitch_time_sec = 0.0
        self.pitch_lock = threading.Lock()

        # Pitch angle history for interpolation (timestamp, pitch)
        self.pitch_history = deque(maxlen=100)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )
        self.pitch_sub = self.create_subscription(
            Float32, 'lidar_pitch', self.pitch_callback, 10
        )

        # Timer for periodic saving
        self.save_timer = self.create_timer(SAVE_INTERVAL, self.save_callback)

        self.get_logger().info(
            f"PointCloud Accumulator started. Saving every {SAVE_INTERVAL}s to {OUTPUT_DIR}"
        )

    def pitch_callback(self, msg):
        """Store pitch angle with current time."""
        now = self.get_clock().now()
        time_sec = now.nanoseconds / 1e9

        with self.pitch_lock:
            self.latest_pitch = msg.data
            self.pitch_time_sec = time_sec
            self.pitch_history.append((time_sec, msg.data))

    def get_pitch_for_time(self, scan_time_sec):
        """
        Get interpolated pitch angle for a given scan timestamp.
        Uses linear interpolation between recorded pitch values.
        """
        with self.pitch_lock:
            if len(self.pitch_history) == 0:
                return self.latest_pitch

            history = list(self.pitch_history)

        # Find bracketing pitch samples
        before = None
        after = None

        for t, p in history:
            if t <= scan_time_sec:
                before = (t, p)
            elif t > scan_time_sec and after is None:
                after = (t, p)
                break

        if before is None and after is None:
            return 0.0
        elif before is None:
            return after[1]
        elif after is None:
            return before[1]
        else:
            # Linear interpolation
            t0, p0 = before
            t1, p1 = after
            if t1 == t0:
                return p0
            alpha = (scan_time_sec - t0) / (t1 - t0)
            return p0 + alpha * (p1 - p0)

    def scan_callback(self, msg):
        """
        Process incoming laser scan:
        1. Get pitch angle for this scan's timestamp
        2. Convert polar (range, angle) to cartesian (x, y)
        3. Rotate by pitch to get 3D (x, y, z)
        4. Accumulate points
        """
        # Get scan timestamp
        scan_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

        # Get pitch angle for this scan
        pitch_deg = self.get_pitch_for_time(scan_time_sec)
        pitch_rad = math.radians(pitch_deg)

        # Precompute rotation values
        cos_pitch = math.cos(pitch_rad)
        sin_pitch = math.sin(pitch_rad)

        new_points = []

        # Process each range measurement
        angle = msg.angle_min
        for r in msg.ranges:
            # Skip invalid ranges
            if math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max:
                angle += msg.angle_increment
                continue

            # Convert polar to 2D cartesian (in lidar's horizontal plane)
            # x = forward, y = left
            x_2d = r * math.cos(angle)
            y_2d = r * math.sin(angle)

            # Rotate around Y axis (pitch) to get 3D coordinates
            # Pitching forward (positive pitch) tilts the scan plane down in front
            # x' = x * cos(pitch)
            # z' = x * sin(pitch)  (z is up)
            x_3d = x_2d * cos_pitch
            y_3d = y_2d  # y unchanged by pitch rotation
            z_3d = x_2d * sin_pitch

            new_points.append((x_3d, y_3d, z_3d))

            angle += msg.angle_increment

        # Add to accumulated points
        with self.points_lock:
            self.points.extend(new_points)

            # Trim if too many points (keep most recent)
            if len(self.points) > MAX_POINTS:
                self.points = self.points[-MAX_POINTS:]

        self.get_logger().debug(
            f"Added {len(new_points)} points at pitch {pitch_deg:.1f}°, "
            f"total: {len(self.points)}"
        )

    def save_callback(self):
        """Save accumulated points to PLY file."""
        with self.points_lock:
            if len(self.points) == 0:
                self.get_logger().info("No points to save yet.")
                return

            points_to_save = list(self.points)

        # Generate filename with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(OUTPUT_DIR, f'scan_{timestamp}.ply')

        self.save_ply(filename, points_to_save)
        self.get_logger().info(
            f"Saved {len(points_to_save)} points to {filename}"
        )

    def save_ply(self, filename, points):
        """Save points to PLY format."""
        with open(filename, 'w') as f:
            # PLY header
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(points)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("end_header\n")

            # Write points
            for x, y, z in points:
                f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")


def main(args=None):
    rclpy.init(args=args)

    node = PointCloudAccumulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Save any remaining points before shutdown
        node.save_callback()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

