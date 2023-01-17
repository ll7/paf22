#!/usr/bin/env python

"""
This node tests the subscription side of the dummy trajectory message.
It therefore receives a nav_msgs/Path msg.
"""

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

from coordinate_transformation import CoordinateTransformer, GeoRef, \
    quat2heading
import rospy


class PositionPublisher(CompatibleNode):
    """
    Creates a node that publishes the current Pose of the vehicle.
    """

    def __init__(self):
        """
        Constructor
        :return:
        """

        super(PositionPublisher, self).__init__('position_publisher')
        self.loginfo("PositionPublisher node started")

        self.transformer = CoordinateTransformer()
        gps_ref = GeoRef.TOWN12
        lat0 = gps_ref.value[0]
        lon0 = gps_ref.value[1]
        h0 = gps_ref.value[2]
        self.transformer.set_gnss_ref(lat0, lon0, h0)

        # current_pos ist the final PoseStamped that will be published
        self.current_pos: PoseStamped = PoseStamped()

        # most upto date sensor data (GPS converted to global XYZ)
        self.current_gps_pos: NavSatFix = NavSatFix()
        self.current_imu: Imu = Imu()
        self.current_heading: float = 0.0

        # basic info
        self.role_name = self.get_param("role_name", "hero")
        self.control_loop_rate = self.get_param("control_loop_rate", "0.05")
        self.publish_loop_rate = 0.05  # 20Hz rate like the sensors

        # Subscriber
        self.gnss_subscriber = self.new_subscription(
            NavSatFix,
            "/carla/" + self.role_name + "/GPS",
            self.update_gps_data,
            qos_profile=1)

        self.imu_subscriber = self.new_subscription(
            Imu,
            "/carla/" + self.role_name + "/IMU",
            self.update_imu_data,
            qos_profile=1)

        # Publisher
        self.pos_publisher = self.new_publisher(
            PoseStamped,
            "/carla/" + self.role_name + "/current_pos",
            qos_profile=1)

        self.heading_publisher = self.new_publisher(
            Float32,
            "/carla/" + self.role_name + "/current_heading",
            qos_profile=1)

    def update_imu_data(self, data: Imu):
        self.current_imu = data

    def update_gps_data(self, data: NavSatFix):
        # self.loginfo("updating gps data")
        # lat = data.latitude
        # lon = data.longitude
        # alt = data.altitude
        self.current_gps_pos = data
        # x, y, z = self.transformer.gnss_to_xyz(lat, lon, alt)
        #
        # self.current_gps_pos.header.stamp = rospy.Time.now()
        # self.current_gps_pos.header.frame_id = "global"
        #
        # self.current_gps_pos.pose.position.x = x
        # self.current_gps_pos.pose.position.y = y
        # self.current_gps_pos.pose.position.z = z
        #
        # self.current_gps_pos.pose.orientation = self.current_imu.orientation

    def update_current_pos(self):
        """
        Updates the current position based on the most upto date
        IMU, Speedometer and GNSS sensor data.
        :return:
        """
        # self.loginfo("updating pos data")
        lat = self.current_gps_pos.latitude
        lon = self.current_gps_pos.longitude
        alt = self.current_gps_pos.altitude
        x, y, z = self.transformer.gnss_to_xyz(lat, lon, alt)
        self.current_heading = quat2heading(self.current_imu)[0]
        # self.loginfo(self.current_heading)

        temp_pose: PoseStamped = PoseStamped()

        # todo: add filtered position update
        # -> for testing without filters
        # temp_pose = self.current_gps_pos.pose
        # <-

        temp_pose.header.stamp = rospy.Time.now()
        temp_pose.header.frame_id = "global"
        temp_pose.pose.position.x = x
        temp_pose.pose.position.y = y
        temp_pose.pose.position.z = z
        temp_pose.pose.orientation = self.current_imu.orientation
        return temp_pose

    def publish_current_pos(self):
        """
        Calls for an update of the current position,
        and then publishes the current position.
        :return:
        """
        # self.loginfo("publishing data")
        temp_pos = self.update_current_pos()
        self.pos_publisher.publish(temp_pos)
        self.heading_publisher.publish(self.current_heading)

    def run(self):
        """
        Control loop

        :return:
        """

        def loop(timer_event=None):
            self.publish_current_pos()

        self.new_timer(self.publish_loop_rate, loop)
        self.spin()


def main(args=None):
    """
    main function

    :param args:
    :return:
    """

    roscomp.init("position_publisher", args=args)
    try:
        node = PositionPublisher()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
