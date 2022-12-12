"""
Helper functions for calculations
inspired by: PSAF 2 WS 20/21 (Acting package)
"""

import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Path


def calc_path_yaw(path: Path, idx: int) -> float:
    """
    Calculates the path yaw.

    Args:
        path (Path): The path to calulate the yaw on
        idx (int): The pose index

    Returns:
        float: [description]
    """
    if idx >= len(path) - 1:
        raise RuntimeError("no target found")

    point_current = path[idx]
    point_next = path[idx + 1]
    angle = math.atan2(point_next.y - point_current.y, point_next.x -
                       point_current.x)
    return normalize_angle(angle)


def normalize_angle(angle: float) -> float:
    """
    Normalizes an angle to [-pi, pi].

    Args:
        angle (float): The angle to normalize

    Returns:
        float: Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


def calc_egocar_yaw(pose: PoseStamped) -> float:
    """
    Calculates the yaw of the ego vehicle.

    Args:
        pose (Pose): The current pose of the ego vehicle

    Returns:
        float: The normalized yaw
    """
    quaternion = (pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w)
    _, _, yaw = euler_from_quaternion(quaternion)
    return normalize_angle(yaw)
