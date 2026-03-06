"""
mock_test.launch.py
===================
Runs the full pipeline with all mock data — no real hardware needed.

Launches:
  1. mock_gps_imu    — fake GPS + IMU, moves rover based on motor commands
  2. mock_vision     — fake obstacle detections, cycles through 4 scenarios
  3. rover_nav       — the actual navigation node under test
  4. foxglove_bridge — visualisation (connect at ws://localhost:8765)

Usage:
    ros2 launch avoidance_rerouting mock_test.launch.py

Override goal with:
    ros2 launch avoidance_rerouting mock_test.launch.py \
        goal_lat:=37.42900 goal_lon:=-122.16800
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            "goal_lat",
            default_value="37.42850",
            description="Goal latitude in decimal degrees",
        ),
        DeclareLaunchArgument(
            "goal_lon",
            default_value="-122.16880",
            description="Goal longitude in decimal degrees",
        ),

        # 1. Mock GPS + IMU
        Node(
            package    = "avoidance_rerouting",
            executable = "mock_gps_imu",
            name       = "mock_gps_imu",
            output     = "screen",
        ),

        # 2. Mock vision obstacles
        Node(
            package    = "avoidance_rerouting",
            executable = "mock_vision",
            name       = "mock_vision",
            output     = "screen",
        ),

        # 3. Navigation node under test
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

        # 4. Foxglove visualisation
        Node(
            package    = "foxglove_bridge",
            executable = "foxglove_bridge",
            name       = "foxglove_bridge",
            output     = "screen",
            parameters = [{"port": 8765}],
        ),
    ])