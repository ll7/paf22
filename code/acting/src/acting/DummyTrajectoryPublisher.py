#!/usr/bin/env python

"""
This node publishes a dummy trajectory between two points,
when receiving a message containing two points.
"""

import ros_compatibility as roscomp
from msg import DummyTrajectoryMsg, DummyTrajectoryRequest
from roscomp.node import CompatibleNode
from trajectory_interpolation import interpolate_route


class DummyTrajectoryPub(CompatibleNode):

    """
    Creates a node capable of publishing a dummy trajectory,
    between to points
    """

    def __int__(self):
        """
        Constructor
        :return:
        """

        super(DummyTrajectoryPub, self).__init("dummy_trajectory_sub")

        # basic info
        self.role_name = "Dummy Trajectory"
        self.target_trajectory = []
        self.pub_trajectory_msg = DummyTrajectoryMsg()

        # request for a new interpolated dummy trajectory
        self.dummy_trajectory_request_subscriber = self.new_subscription(
            DummyTrajectoryRequest,
            "/carla/"+self.role_name+"/dummy_trajectory_request",
            self.dummy_trajectory_requested,
            qos_profile=1)

        # publisher for the current trajectory
        self.dummy_trajectory_publisher = self.new_publisher(
            DummyTrajectoryMsg,
            "/carla/"+self.role_name+"/dummy_trajectory",
            qos_profile=1)

    def dummy_trajectory_requested(self, dummy_trajectory_request):
        """
        Stores the new trajectory
        :param dummy_trajectory_request: start and end waypoints
        :return:
        """
        self.target_trajectory = interpolate_route(dummy_trajectory_request.wp_list)
        self.pub_trajectory_msg.wp_list = self.target_trajectory

    def run(self):
        """
        Control loop

        :return:
        """

        def loop(timer_event=None):
            pub_trajectory_message = DummyTrajectoryMsg()

            pub_trajectory_message.wp_list = self.target_trajectory

            self.dummy_trajectory_publisher.publish(pub_trajectory_message)
        self.spin()

def main(args=None):
    """
    main function

    :param args:
    :return:
    """

    roscomp.init("dummy_trajectory_sub")
    try:
        node = DummyTrajectoryPub()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
