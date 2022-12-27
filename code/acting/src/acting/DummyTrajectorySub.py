#!/usr/bin/env python

"""
This node tests the subscription side of the dummy trajectory message.
It therefore receives a nav_msgs/Path msg.
"""

import ros_compatibility as roscomp
# import matplotlib.pyplot as plt
from ros_compatibility.node import CompatibleNode
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from coordinate_transformation import CoordinateTransformer


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

        self.transformer = CoordinateTransformer
        self.transformer.__init__(self)

        gps_lat_ref, gps_lon_ref = 35.25000, -101.87500
        m_h_ref = 331.7265

        self.transformer.set_gnss_ref(self, gps_lat_ref, gps_lon_ref, m_h_ref)

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

        self.pos_counter = 0
        self.pos_list = []
        self.gnss_subscriber = self.new_subscription(
            NavSatFix,
            "/carla/" + self.role_name + "/GPS",
            self.output_gps_2_xyz,
            qos_profile=1)

    def output_gps_2_xyz(self, data: NavSatFix):
        if self.pos_counter % 20 == 0:
            lat = data.latitude
            lon = data.longitude
            h = data.altitude
            x, y, z = self.transformer.gnss_to_xyz(self, lat, lon, h)
            self.loginfo(f"x: {x}\t y: {y}\t z:{z}")
            # self.pos_list.append((x, y, z))

    '''
        x, y, z = geodetic_to_ecef(35.20171220851982,
                                   -101.86417900962616,
                                   372.98386)
        self.loginfo("x: ")
        self.loginfo(x)
        self.loginfo("y: ")
        self.loginfo(y)
        self.loginfo("z: ")
        self.loginfo(z)

    '''

    '''
    def output_xodr_map(self, data: String):
        self.loginfo("Saving XODR String")
        text_file = open("xodr_string.txt", "w")
        text_file.write(data.data)
        text_file.close()
    '''

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
