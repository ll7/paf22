"""
A module for interpolating routes
Source: https://github.com/ll7/paf21-1
"""

from math import dist as euclid_dist, floor, sqrt, sin, cos
from typing import List, Tuple


def points_to_vector(p_1: Tuple[float, float],
                     p_2: Tuple[float, float]) -> Tuple[float, float]:
    """
    Create the vector starting at p1 and ending at p2
    :param p_1: Start point
    :param p_2: End point
    :return: Vector from p1 to p2
    """
    return p_2[0] - p_1[0], p_2[1] - p_1[1]


def vector_len(vec: Tuple[float, float]) -> float:
    """
    Compute the given vector's length
    :param vec: vector v as a tuple (x, y)
    :return: length of vector v
    """
    return sqrt(vec[0]**2 + vec[1]**2)


def add_vector(v_1: Tuple[float, float],
               v_2: Tuple[float, float]) -> Tuple[float, float]:
    """
    Add the two given vectors
    :param v_1: first vector
    :param v_2: second vector
    :return: sum of first and second vector
    """
    """Add the given vectors"""
    return v_1[0] + v_2[0], v_1[1] + v_2[1]


def rotate_vector(vector: Tuple[float, float],
                  angle_rad: float) -> Tuple[float, float]:
    """
    Rotate the given vector by an angle
    :param vector: vector
    :param angle_rad: angle of rotation
    :return: rotated angle
    """
    return (cos(angle_rad) * vector[0] - sin(angle_rad) * vector[1],
            sin(angle_rad) * vector[0] + cos(angle_rad) * vector[1])


def linear_interpolation(start: Tuple[float, float], end: Tuple[float, float],
                         interval_m: float) -> List[Tuple[float, float]]:
    """
    Interpolate linearly between start and end,
    with a minimal distance of interval_m between points.
    :param start: starting point
    :param end: target point
    :param interval_m: min distance between interpolated points, if possible
    :return: interpolated list of points between start / end
    """

    distance = euclid_dist(start, end)
    vector = points_to_vector(start, end)

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


def _clean_route_duplicates(route: List[Tuple[float, float]],
                            min_dist: float) -> List[Tuple[float, float]]:
    """
    Remove duplicates in the given List of tuples,
    if the distance between them is less than min_dist.
    :param route: list of points that should be cleaned up
    :param min_dist: minimal allowed distance between points
    :return: cleaned up list of points
    """
    cleaned_route = [route[0]]
    for next_p in route[1:]:
        if euclid_dist(cleaned_route[-1], next_p) >= min_dist:
            cleaned_route.append(next_p)
        else:
            print(next_p)
    return cleaned_route


def interpolate_route(orig_route: List[Tuple[float, float]], interval_m=0.5):
    """
    Interpolate the given route with points inbetween,
    holding the specified distance interval threshold.
    :param orig_route: route
    :param interval_m: minimum distance between two points
    :return: interpolated rout
    """

    orig_route = _clean_route_duplicates(orig_route, 0.1)
    route = []
    for index in range(len(orig_route) - 1):
        waypoints = linear_interpolation(orig_route[index],
                                         orig_route[index + 1], interval_m)
        route.extend(waypoints)

    route = route + [orig_route[-1]]
    return route
