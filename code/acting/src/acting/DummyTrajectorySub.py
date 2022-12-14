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
from geometry_msgs.msg import PoseStamped
from coordinate_transformation import CoordinateTransformer, GeoRef
import rospy


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

        self.transformer = CoordinateTransformer()
        gps_ref = GeoRef.TOWN12
        lat0 = gps_ref.value[0]
        lon0 = gps_ref.value[1]
        h0 = gps_ref.value[2]
        self.transformer.set_gnss_ref(lat0, lon0, h0)

        self.current_pos = PoseStamped()

        # basic info
        self.role_name = self.get_param("role_name", "ego_vehicle")
        self.control_loop_rate = self.get_param("control_loop_rate", "0.05")

        # initial waypoints
        self.start_wp = (0.0, 0.0)
        self.target_wp = (100.0, 0.0)
        self.target_trajectory = [self.start_wp, self.target_wp]

        # Subscriber
        self.dummy_trajectory_subscriber = self.new_subscription(
            Path,
            "/carla/" + self.role_name + "/trajectory",
            self.update_trajectory,
            qos_profile=1)

        self.pos_counter = 0
        self.pos_average = [0, 0, 0]
        self.gnss_subscriber = self.new_subscription(
            NavSatFix,
            "/carla/" + self.role_name + "/GPS",
            self.output_average_gps_2_xyz,
            qos_profile=1)

        # Publisher
        self.pos_publisher = self.new_publisher(
            PoseStamped,
            "/carla/" + self.role_name + "/current_pos",
            qos_profile=1)

    def output_gps_2_xyz(self, data: NavSatFix):
        """
        Transforms GPS coordinates into local coordinates
        using the CoordinateTransformer transformer.
        The current position is then updated and published
        in the PoseStamped format.
        :param data: message according to NavSatFix definition
        :return:
        """
        if self.pos_counter % 4 == 0:
            lat = data.latitude
            lon = data.longitude
            h = data.altitude
            x, y, z = self.transformer.gnss_to_xyz(self, lat, lon, h)
            self.update_pose(x, y, z)
            # self.loginfo(f"x: {x}\t y: {y}\t z:{z}")

        self.pos_counter += 1

    def output_average_gps_2_xyz(self, data: NavSatFix):
        """
        Transforms GPS coordinates into local coordinates
        using the CoordinateTransformer transformer.
        The current position is then updated and published
        in the PoseStamped format.
        :param data: message according to NavSatFix definition
        :return:
        """

        lat = data.latitude
        lon = data.longitude
        h = data.altitude
        x, y, z = self.transformer.gnss_to_xyz(lat, lon, h)

        self.pos_average[0] += x
        self.pos_average[1] += y
        self.pos_average[2] += z

        if self.pos_counter % 5 == 0:
            x1 = self.pos_average[0] / 5
            y1 = self.pos_average[1] / 5
            z1 = self.pos_average[2] / 5
            self.pos_average = [0, 0, 0]
            self.update_pose(x1, y1, z1)
            # self.loginfo(f"x: {x1}\t y: {y1}\t z:{z1}")

        self.pos_counter += 1

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

    def update_pose(self, x, y, z):
        """
        Update the current position and publishes the new position.
        :param x:
        :param y:
        :param z:
        :return:
        """

        temp_pose = PoseStamped()

        temp_pose.header.stamp = rospy.Time.now()
        temp_pose.header.frame_id = "Local Coordinate Frame"

        temp_pose.pose.position.x = x
        temp_pose.pose.position.y = y
        temp_pose.pose.position.z = z

        temp_pose.pose.orientation.x = 0.0
        temp_pose.pose.orientation.y = 0.0
        temp_pose.pose.orientation.z = 0.0
        temp_pose.pose.orientation.w = 0.0

        self.current_pos = temp_pose
        self.pos_publisher.publish(self.current_pos)

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
