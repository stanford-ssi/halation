"""
navigation.launch.py
====================
Launches the full GPS + Vision navigation stack.

Usage (set goal via launch args):
    ros2 launch avoidance_rerouting navigation.launch.py \
        goal_lat:=37.4280 goal_lon:=-122.1690

The rover will start moving as soon as it gets a GPS fix.
Goal can also be updated at runtime:
    ros2 topic pub /goal_gps std_msgs/String \
      '{"data": "{\"lat\": 37.4280, \"lon\": -122.169}"}'
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ── Launch arguments ─────────────────────────────────────────────────
        DeclareLaunchArgument(
            "goal_lat",
            default_value="0.0",
            description="Goal latitude in decimal degrees (e.g. 37.4280)",
        ),
        DeclareLaunchArgument(
            "goal_lon",
            default_value="0.0",
            description="Goal longitude in decimal degrees (e.g. -122.1690)",
        ),

        # ── Vision detection node ─────────────────────────────────────────────
        # Camera → GroundingDINO → /obstacle_detections
        Node(
            package    = "avoidance_rerouting",
            executable = "vision_detection",
            name       = "vision_detection",
            output     = "screen",
        ),

        # ── Navigation node ───────────────────────────────────────────────────
        # GPS + IMU + /obstacle_detections → /motor_vector
        Node(
            package    = "avoidance_rerouting",
            executable = "rover_nav",
            name       = "rover_nav",
            output     = "screen",
            parameters = [{
                "goal_lat": LaunchConfiguration("goal_lat"),
                "goal_lon": LaunchConfiguration("goal_lon"),
            }],
        ),

        # ── Foxglove bridge ───────────────────────────────────────────────────
        # Connect Foxglove Studio to ws://ROVER_IP:8765 to visualise
        Node(
            package    = "foxglove_bridge",
            executable = "foxglove_bridge",
            name       = "foxglove_bridge",
            output     = "screen",
            parameters = [{"port": 8765}],
        ),

        # ── Teleop bridge ───────────────────────────────────────────────────
        Node(
            package    = "rosbridge_server",
            executable = "rosbridge_websocket",
            name       = "rosbridge_websocket",
            output     = "screen",
            parameters = [{"port": 9095}],
        ),

    ])