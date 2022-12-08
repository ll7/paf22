#!/usr/bin/env python

"""
This node publishes a dummy trajectory between two points,
when receiving a message containing two points.
"""


from acting.srv import DummyTrajectorySrv, DummyTrajectorySrvResponse
from acting.src.acting.trajectory_interpolation import interpolate_route
import rospy


def handle_dummy_trajectory_request(req):
    """
    interpolate between two waypoints specified in req and return trajectory
    :param req: start_wp and target_wp
    :return: trajectory between start_wp and target_wp
    """

    start_wp = (req.start_wp_x, req.start_wp_y)
    target_wp = (req.target_wp_x, req.target_wp_y)
    wp_list = [start_wp, target_wp]
    trajectory = interpolate_route(wp_list, 0.2)

    return DummyTrajectorySrvResponse(trajectory)


def dummy_trajectory_server():
    rospy.init_node("dummy_trajectory_server")
    s = rospy.Service("dummy_trajectory", DummyTrajectorySrv, handle_dummy_trajectory_request)
    rospy.spin()


if __name__ == "__main__":
    dummy_trajectory_server()

