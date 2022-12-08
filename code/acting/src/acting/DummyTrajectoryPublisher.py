#!/usr/bin/env python

"""
This node publishes a dummy trajectory between two points,
when receiving a message containing two points.
"""

import ros_compatibility as roscomp
from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from msg import DummyTrajectoryMsg, DummyTrajectoryRequest
from ros_compatibility.node import CompatibleNode
from trajectory_interpolation import interpolate_route
import rospy


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
        super(DummyTrajectoryPub, self).__init("trajectory_pub")

        # basic info
        self.role_name = "Dummy Trajectory"

        self.current_trajectory = []
        self.path_msg = Path()
        self.path_msg.header.stamp = rospy.Time.now()
        self.path_msg.header.frame_id = "Frame ID Path"

        initial_trajectory = [
            (983.5, -5373.2),
            (1083.5, -5273.2),
            (1083.5, -5273.2)
        ]
        self.updated_trajectory(self, initial_trajectory)

        # request for a new interpolated dummy trajectory
        self.dummy_trajectory_request_subscriber = self.new_subscription(
            DummyTrajectoryRequest,
            "/carla/"+self.role_name+"/dummy_trajectory_request",
            self.dummy_trajectory_requested,
            qos_profile=1)

        # publisher for the current trajectory
        self.trajectory_publisher = self.new_publisher(
            Path,
            "/carla/"+self.role_name+"/trajectory",
            qos_profile=1)

    def updated_trajectory(self, target_trajectory):
        self.current_trajectory = interpolate_route(target_trajectory, 5)
        self.path_msg.header.stamp = rospy.Time.now()
        self.path_msg.header.frame_id = "Frame ID Path Update"

        #clear old waypoints
        self.path_msg.poses.clear()

        for wp in self.current_trajectory:
            pos = PoseStamped()
            pos.header.stamp = rospy.Time.now()
            pos.header.frame_id = "Frame ID Pos"

            pos.pose.position.x = wp[1]
            pos.pose.position.y = wp[2]

            # currently not used therfore zeros
            pos.pose.position.z = 0
            pos.pose.orientation.x = 0
            pos.pose.orientation.y = 0
            pos.pose.orientation.z = 0
            pos.pose.orientation.w = 0

            self.path_msg.poses.append(pos)

    """
    def dummy_trajectory_requested(self, dummy_trajectory_request):
        ""
        Stores the new trajectory
        :param dummy_trajectory_request: start and end waypoints
        :return:
        ""
        self.target_trajectory = interpolate_route(dummy_trajectory_request.wp_list)
        self.pub_trajectory_msg.wp_list = self.target_trajectory
    """

    def run(self):
        """
        Control loop

        :return:
        """

        def loop(timer_event=None):
            # Continuously update path
            self.trajectory_publisher.publish(self.path_msg)

        self.spin()



def main(args=None):
    """
    main function

    :param args:
    :return:
    """

    roscomp.init("trajectory_pub")
    try:
        node = DummyTrajectoryPub()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
