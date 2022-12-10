"""A module for interpolating routes"""

from math import dist as euclid_dist, floor, sqrt, sin, cos
from typing import List, Tuple


def points_to_vector(p_1: Tuple[float, float],
                     p_2: Tuple[float, float]) -> Tuple[float, float]:
    """Create the vector starting at p1 and ending at p2"""
    return p_2[0] - p_1[0], p_2[1] - p_1[1]


def vector_len(vec: Tuple[float, float]) -> float:
    """Compute the given vector's length"""
    return sqrt(vec[0]**2 + vec[1]**2)


def add_vector(v_1: Tuple[float, float],
               v_2: Tuple[float, float]) -> Tuple[float, float]:
    """Add the given vectors"""
    return v_1[0] + v_2[0], v_1[1] + v_2[1]


def scale_vector(vector: Tuple[float, float],
                 new_len: float) -> Tuple[float, float]:
    """Amplify the length of the given vector"""
    old_len = vector_len(vector)
    if old_len == 0:
        return (0, 0)
    scaled_vector = (vector[0] * new_len / old_len,
                     vector[1] * new_len / old_len)
    return scaled_vector


def rotate_vector(vector: Tuple[float, float],
                  angle_rad: float) -> Tuple[float, float]:
    """Rotate the given vector by an angle"""
    return (cos(angle_rad) * vector[0] - sin(angle_rad) * vector[1],
            sin(angle_rad) * vector[0] + cos(angle_rad) * vector[1])


def linear_interpolation(start: Tuple[float, float], end: Tuple[float, float],
                         interval_m: float) -> List[Tuple[float, float]]:
    """Interpolate linearly between the given start / end point
    by putting points according to the interval specified."""

    distance = euclid_dist(start, end)
    vector = (end[0] - start[0], end[1] - start[1])

    steps = max(1, floor(distance / interval_m))
    exceeds_interval_cap = distance > interval_m
    step_vector = (vector[0] / steps if exceeds_interval_cap else vector[0],
                   vector[1] / steps if exceeds_interval_cap else vector[1])

    lin_points = [(start[0], start[1])]
    for i in range(1, steps):
        lin_points.append(
            (start[0] + step_vector[0] * i,
             start[1] + step_vector[1] * i)
        )

    return lin_points


'''
def circular_interpolation(start: Tuple[float, float], end: Tuple[float, float],
                           arc_radius: float) -> List[Tuple[float, float]]:
    """Interpolate points between start / end point
    on top of the circular arc given by the arc radius."""

    step_size = 2.0
    sign = -1 if arc_radius < 0 else 1
    arc_radius = abs(arc_radius)

    # determine the circular angle of the arc
    angle = asin((euclid_dist(start, end) / 2) / arc_radius) * 2

    # construct the mid-perpendicular of |start, end| to determine the center
    conn_middle = ((start[0] + end[0]) / 2, (start[1] + end[1]) / 2)
    center_off = sqrt(pow(arc_radius, 2) - pow(euclid_dist(start, end) / 2, 2))
    mid_perpend = rotate_vector(points_to_vector(start, end), pi/2 * sign)
    circle_center = add_vector(conn_middle,
                              scale_vector(mid_perpend, center_offset)
                              )

    # partition the arc into steps (-> interpol. geometries)
    arc_circumference = arc_radius * angle         # (r * 2 pi) * (angle / 2 pi)
    num_steps = int(arc_circumference / step_size) + 1  # each step < step size

    # compute the interpolated points on the circle arc
    vec = points_to_vector(circle_center, start)
    rot_angles = [angle * (i / num_steps) for i in range(num_steps+1)]
    points = [add_vector(
                        circle_center,
                        rotate_vector(vec, rot * sign)
                        )
                        for rot in rot_angles]

    return points
'''


def _clean_route_duplicates(route: List[Tuple[float, float]],
                            min_dist: float) -> List[Tuple[float, float]]:
    """
    Remove duplicates in the given List of tuples, if the distance between them
    is less than min_dist.
    :param route: list of points that should be cleaned up
    :param min_dist: minimal allowed distance between points
    :return: cleaned list of points
    """
    cleaned_route = [route[0]]
    for next_p in route[1:]:
        if euclid_dist(cleaned_route[-1], next_p) >= min_dist:
            cleaned_route.append(next_p)
    return cleaned_route


def interpolate_route(orig_route: List[Tuple[float, float]], interval_m=0.5):
    """Interpolate the given route with points inbetween,
    holding the specified distance interval threshold."""

    orig_route = _clean_route_duplicates(orig_route, 0.01)
    route = []
    for index in range(len(orig_route) - 1):
        waypoints = linear_interpolation(orig_route[index],
                                         orig_route[index + 1], interval_m)
        route.extend(waypoints)

    route = route + [orig_route[-1]]
    return _clean_route_duplicates(route, 0.1)
