"""Adapters that let the operator console drive a real ROS graph.

Only the pieces that need no ROS installed are exported here. console_node
imports rclpy, so importing it from this file would make the whole package
unimportable on a machine that has the console but not the robot.
"""

from .map_provider import GridSnapshot, RobotPose, build_map
from .occupancy_image import data_url, grayscale_png
from .safety_adapter import parse_state

__all__ = [
    "GridSnapshot",
    "RobotPose",
    "build_map",
    "data_url",
    "grayscale_png",
    "parse_state",
]
