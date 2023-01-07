"""
Helper functions for calculations
inspired by: PSAF 2 WS 20/21 (Acting package)
"""

import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Path
from scipy.spatial.transform import Rotation


# todo: docs
def vectors2angle(x1: float, y1: float, x2: float, y2: float) -> float:
    v1 = [x1, y1]
    v2 = [x2, y2]

    cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    alpha = np.arccos(cos_angle)

    return math.degrees(alpha)


def get_vector_direction(x1, y1, x2, y2) -> float:
    theta = math.atan(x2 - x1 / y2 - y1)
    return math.degrees(theta)


def quaternion2heading(x: float, y: float, z: float, w: float) -> float:
    quaternion = (x, y, z, w)
    rot = Rotation.from_quat(quaternion)
    rot_euler = rot.as_euler("xyz", degrees=True)
    return rot_euler[2]


def heading2quaternion(heading: float) -> (float, float, float, float):
    rot = Rotation.from_euler("xyz", (0, 0, heading), degrees=True)
    quat = rot.as_quat()
    return quat[0], quat[1], quat[2], quat[3]


def calc_path_yaw(path: Path, idx: int) -> float:
    """
    Calculates the path yaw.
    Args:
        path (Path): The path to calulate the yaw on
        idx (int): The pose index
    Returns:
        float: [description]
    """
    if idx >= len(path.poses) - 1:
        raise RuntimeError("no target found")

    point_current: PoseStamped
    point_current = path.poses[idx]
    point_next: PoseStamped
    point_next = path.poses[idx + 1]
    angle = math.atan2(point_next.pose.position.y
                       - point_current.pose.position.y,
                       point_next.pose.position.x
                       - point_current.pose.position.x)
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
    quaternion = (pose.pose.orientation.x, pose.pose.orientation.y,
                  pose.pose.orientation.z, pose.pose.orientation.w)
    _, _, yaw = euler_from_quaternion(quaternion)
    return normalize_angle(yaw)
