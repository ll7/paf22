#!/usr/bin/env python

"""
This node publishes a dummy trajectory between two points,
when receiving a message containing two points.
"""

import ros_compatibility as roscomp
from msg import DummyTrajectoryMsg
from ros_compatibility.node import CompatibleNode

class DummyTrajectorySub(CompatibleNode):

    """
    Creates a node capable of publishing a dummy trajectory,
    between to points
    """

    def __int__(self):
        """
        Constructor
        :return:
        """

        super(DummyTrajectorySub, self).__init("dummy_trajectory_sub")

        #basic info
        self.role_name = "Dummy Trajectory"

        # initial waypoints
        self.info.start_wp = (0.0, 0.0)
        self.info.target_wp = (100.0, 0.0)
        self.target_trajectory = [self.info.start_wp, self.info.target_wp]

        self.dummy_trajectory_subscriber = self.new_subscription(
            DummyTrajectoryMsg,
            "/carla/"+self.role_name+"/dummy_trajectory",
            self.update_trajectory,
            qos_profile=1)

    def update_trajectory(self, dummy_trajectory):
        """
        Stores the new trajectory
        :param dummy_trajectory:
        :return:
        """
        self.target_trajectory = dummy_trajectory.wp_list

    def run(self):
        """
        Control loop

        :return:
        """
        self.spin()

def main(args=None):
    """
    main function

    :param args:
    :return:
    """

    roscomp.init("dummy_trajectory_sub")
    try:
        node = DummyTrajectorySub()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()

if __name__ == "__main__":
    main()
