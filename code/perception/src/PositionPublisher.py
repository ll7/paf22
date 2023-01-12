#!/usr/bin/env python

"""
This node tests the subscription side of the dummy trajectory message.
It therefore receives a nav_msgs/Path msg.
"""

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped, Pose
from coordinate_transformation import CoordinateTransformer, GeoRef
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
        self.current_gps_pos: PoseStamped = PoseStamped()
        self.current_imu: Imu = Imu()

        # basic info
        self.role_name = self.get_param("role_name", "ego_vehicle")
        self.control_loop_rate = self.get_param("control_loop_rate", "0.05")
        self.publish_loop_rate = 0.05  # 20Hz rate like the sensors

        # Subscriber
        self.pos_counter = 0
        self.pos_average = [0, 0, 0]
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

    def update_imu_data(self, data: Imu):
        self.current_imu = data

    def update_gps_data(self, data: NavSatFix):
        self.loginfo("updating gps data")
        lat = data.latitude
        lon = data.longitude
        alt = data.altitude
        x, y, z = self.transformer.gnss_to_xyz(lat, lon, alt)

        self.current_gps_pos.header.stamp = rospy.Time.now()
        self.current_gps_pos.header.frame_id = "Global XYZ frame"

        self.current_gps_pos.pose.position.x = x
        self.current_gps_pos.pose.position.y = y
        self.current_gps_pos.pose.position.z = z

        self.current_gps_pos.pose.orientation.x = 0
        self.current_gps_pos.pose.orientation.y = 0
        self.current_gps_pos.pose.orientation.z = 0
        self.current_gps_pos.pose.orientation.w = 0

    def update_current_pos(self):
        """
        Updates the current position based on the most upto date
        IMU, Speedometer and GNSS sensor data.
        :return:
        """
        self.loginfo("updating pos data")
        temp_pose: Pose

        # todo: add filtered position update
        # -> for testing without filters
        temp_pose = self.current_gps_pos.pose
        # <-

        self.current_pos.header.stamp = rospy.Time.now()
        self.current_pos.header.frame_id = "Global XYZ frame"

        self.current_pos.pose = temp_pose

    def publish_current_pos(self):
        """
        Calls for an update of the current position,
        and then publishes the current position.
        :return:
        """
        self.loginfo("publishing data")
        self.update_current_pos()
        self.pos_publisher.publish(self.current_pos)

#    def output_gps_2_xyz(self, data: NavSatFix):
#        """
#        Transforms GPS coordinates into local coordinates
#        using the CoordinateTransformer transformer.
#        The current position is then updated and published
#        in the PoseStamped format.
#        :param data: message according to NavSatFix definition
#        :return:
#        """
#        if self.pos_counter % 4 == 0:
#            lat = data.latitude
#            lon = data.longitude
#            h = data.altitude
#            x, y, z = self.transformer.gnss_to_xyz(lat, lon, h)
#            self.update_pose(x, y, z)
#            # self.loginfo(f"x: {x}\t y: {y}\t z:{z}")
#
#        self.pos_counter += 1
#
#    def output_average_gps_2_xyz(self, data: NavSatFix):
#        """
#        Transforms GPS coordinates into local coordinates
#        using the CoordinateTransformer transformer.
#        The current position is then updated and published
#        in the PoseStamped format.
#        :param data: message according to NavSatFix definition
#        :return:
#        """
#
#        lat = data.latitude
#        lon = data.longitude
#        h = data.altitude
#        x, y, z = self.transformer.gnss_to_xyz(lat, lon, h)
#
#        self.pos_average[0] += x
#        self.pos_average[1] += y
#        self.pos_average[2] += z
#
#        if self.pos_counter % 5 == 0:
#            x1 = self.pos_average[0] / 5
#            y1 = self.pos_average[1] / 5
#            z1 = self.pos_average[2] / 5
#            self.pos_average = [0, 0, 0]
#            self.update_pose(x1, y1, z1)
#            # self.loginfo(f"x: {x1}\t y: {y1}\t z:{z1}")
#
#        self.pos_counter += 1
#
#    def update_pose(self, x, y, z):
#        """
#        Update the current position and publishes the new position.
#        :param x:
#        :param y:
#        :param z:
#        :return:
#        """
#
#        temp_pose = PoseStamped()
#
#        temp_pose.header.stamp = rospy.Time.now()
#        temp_pose.header.frame_id = "Global XYZ Frame"
#
#        temp_pose.pose.position.x = x
#        temp_pose.pose.position.y = y
#        temp_pose.pose.position.z = z
#
#        temp_pose.pose.orientation.x = 0.0
#        temp_pose.pose.orientation.y = 0.0
#        temp_pose.pose.orientation.z = 0.0
#        temp_pose.pose.orientation.w = 0.0
#
#        self.current_pos = temp_pose
#        self.pos_publisher.publish(self.current_pos)

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
