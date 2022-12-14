#!/usr/bin/env python

"""
This node publishes a dummy trajectory between predefined points.
"""

import ros_compatibility as roscomp
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from ros_compatibility.node import CompatibleNode
from trajectory_interpolation import interpolate_route
import rospy


class DummyTrajectoryPub(CompatibleNode):
    """
    Creates a node that publishes an interpolated trajectory between
    predefined points as a nav_msgs/Path message.
    """

    def __init__(self):
        """
        Constructor
        :return:
        """
        super(DummyTrajectoryPub, self).__init__('dummy_trajectory_pub')
        self.loginfo('DummyTrajectoryPub node started')
        # basic info
        self.role_name = self.get_param('role_name', 'ego_vehicle')
        self.control_loop_rate = self.get_param('control_loop_rate', 0.05)

        self.current_trajectory = []
        self.path_msg = Path()
        self.path_msg.header.stamp = rospy.Time.now()
        self.path_msg.header.frame_id = "Frame ID Path"

        # Static trajectory for testing purposes
        initial_trajectory = [
            (985.0, -5373.2),
            (985.0, -5473.2),
            (990.0, -5474.2),
            (990.0, -5524.2),
            (990.0, -5574.2)
        ]
        self.updated_trajectory(initial_trajectory)

        # request for a new interpolated dummy trajectory
        # self.dummy_trajectory_request_subscriber = self.new_subscription(
        #     DummyTrajectoryRequest,
        #     "/carla/"+self.role_name+"/dummy_trajectory_request",
        #     self.dummy_trajectory_requested,
        #     qos_profile=1)

        # publisher for the current trajectory
        self.trajectory_publisher = self.new_publisher(
            Path,
            "/carla/" + self.role_name + "/trajectory",
            qos_profile=1)

    def updated_trajectory(self, target_trajectory):
        """
        Updates the published Path message with the new target trajectory.
        :param: target_trajectory: the new target trajectory to be published
        :return:
        """
        self.current_trajectory = interpolate_route(target_trajectory, 0.5)
        self.path_msg.header.stamp = rospy.Time.now()
        self.path_msg.header.frame_id = "Frame ID Path Update"

        # clear old waypoints
        self.path_msg.poses.clear()

        for wp in self.current_trajectory:
            pos = PoseStamped()
            pos.header.stamp = rospy.Time.now()
            pos.header.frame_id = "Frame ID Pos"

            pos.pose.position.x = wp[0]
            pos.pose.position.y = wp[1]

            # currently not used therefore zeros
            pos.pose.position.z = 0
            pos.pose.orientation.x = 0
            pos.pose.orientation.y = 0
            pos.pose.orientation.z = 0
            pos.pose.orientation.w = 0

            self.path_msg.poses.append(pos)

    def run(self):
        """
        Control loop

        :return:
        """

        def loop(timer_event=None):
            # Continuously update path
            self.trajectory_publisher.publish(self.path_msg)

        self.new_timer(self.control_loop_rate, loop)
        self.spin()


def main(args=None):
    """
    main function

    :param args:
    :return:
    """

    roscomp.init("dummy_trajectory_pub", args=args)
    try:
        node = DummyTrajectoryPub()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
