#!/usr/bin/env python

"""
This node tests the subscription side of the dummy trajectory message.
It therefore receives a nav_msgs/Path msg.
"""

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from nav_msgs.msg import Path


class DummyTrajectorySub(CompatibleNode):
    """
    Creates a node capable of subscribing to a Path msg.
    """

    def __init__(self):
        """
        Constructor
        :return:
        """

        super(DummyTrajectorySub, self).__init__('dummy_trajectory_sub')
        self.loginfo("DummyTrajectorySub node started")

        # basic info
        self.role_name = self.get_param("role_name", "ego_vehicle")
        self.control_loop_rate = self.get_param("control_loop_rate", "0.05")

        # initial waypoints
        self.start_wp = (0.0, 0.0)
        self.target_wp = (100.0, 0.0)
        self.target_trajectory = [self.start_wp, self.target_wp]

        self.dummy_trajectory_subscriber = self.new_subscription(
            Path,
            "/carla/" + self.role_name + "/trajectory",
            self.update_trajectory,
            qos_profile=1)

    def update_trajectory(self, dummy_trajectory):
        """
        Stores the new trajectory
        :param dummy_trajectory: the Path msg with the new trajectory
        :return:
        """

        temp_msg = dummy_trajectory
        # remove all the old waypoints
        self.target_trajectory.clear()

        # save the new waypoints
        for temp_pose in temp_msg.poses:
            x = temp_pose.pose.position.x
            y = temp_pose.pose.position.y
            temp_wp = (x, y)
            self.target_trajectory.append(temp_wp)

    def run(self):
        """
        Control loop

        :return:
        """

        # self.new_timer(self.control_loop_rate, loop)
        self.spin()


def main(args=None):
    """
    main function

    :param args:
    :return:
    """

    roscomp.init("dummy_trajectory_sub", args=args)
    try:
        node = DummyTrajectorySub()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
