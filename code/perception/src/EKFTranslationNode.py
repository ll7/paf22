#!/usr/bin/env python

"""
This node tests the subscription side of the dummy trajectory message.
It therefore receives a nav_msgs/Path msg.
"""

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from coordinate_transformation import CoordinateTransformer, GeoRef


class EKFTranslation(CompatibleNode):
    """
    Translates to the top required by robot_pose_ekf.
    """

    def __init__(self):
        """
        Constructor
        :return:
        """

        super(EKFTranslation, self).__init__('ekf_translation')
        self.loginfo("EKF_Translation node started")

        # basic info
        self.role_name = self.get_param("role_name", "hero")
        self.control_loop_rate = self.get_param("control_loop_rate", "0.05")

        self.transformer = CoordinateTransformer()
        gps_ref = GeoRef.TOWN12
        lat0 = gps_ref.value[0]
        lon0 = gps_ref.value[1]
        h0 = gps_ref.value[2]
        self.transformer.set_gnss_ref(lat0, lon0, h0)

        # Subscriber
        self.imu_subscriber = self.new_subscription(
            Imu,
            "/carla/" + self.role_name + "/IMU",
            self.update_imu_data,
            qos_profile=1)

        self.gps_subscriber = self.new_subscription(
            NavSatFix,
            "/carla/" + self.role_name + "/GPS",
            self.update_gps_data,
            qos_profile=1)

        # Publisher
        # 2D Odometry (Maybe Speedometer?)
        self.ekf_odom_publisher = self.new_publisher(
            Odometry,
            "/odom",
            qos_profile=1)

        # IMU
        self.ekf_imu_publisher = self.new_publisher(
            Imu,
            "/imu_data",
            qos_profile=1)

        # 3D Odomoetry (GPS)
        self.ekf_vo_publisher = self.new_publisher(
            Odometry,
            "/vo",
            qos_profile=1)

    def update_imu_data(self, data: Imu):
        # imu_data = Imu()
        #
        # imu_data.header.stamp = data.header.stamp
        # imu_data.header.frame_id = "base_footprint"
        #
        # imu_data.orientation.x = data.orientation.x
        # imu_data.orientation.y = data.orientation.y
        # imu_data.orientation.z = data.orientation.z
        # imu_data.orientation.w = data.orientation.w
        # imu_data.orientation_covariance = data.orientation_covariance
        #
        # imu_data.angular_velocity.x = data.angular_velocity.x
        # imu_data.angular_velocity.y = data.angular_velocity.y
        # imu_data.angular_velocity.z = data.angular_velocity.z
        # imu_data.angular_velocity_covariance = \
        #     data.angular_velocity_covariance
        #
        # imu_data.linear_acceleration.x = data.linear_acceleration.x
        # imu_data.linear_acceleration.y = data.linear_acceleration.y
        # imu_data.linear_acceleration.z = data.linear_acceleration.z
        # data_lac = data.linear_acceleration_covariance
        # imu_data.linear_acceleration_covariance = data_lac
        #
        # self.ekf_imu_publisher.publish(imu_data)
        self.ekf_imu_publisher.publish(data)

    def update_gps_data(self, data: NavSatFix):
        odom_msg = Odometry()

        odom_msg.header.stamp = data.header.stamp
        odom_msg.header.frame_id = "global"

        # Covariance todo: needs tweaking
        cov_x = 1
        cov_y = 1
        cov_z = 1

        lat = data.latitude
        lon = data.longitude
        alt = data.altitude
        x, y, z = self.transformer.gnss_to_xyz(lat, lon, alt)

        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = z

        odom_msg.pose.pose.orientation.x = 1
        odom_msg.pose.pose.orientation.y = 0
        odom_msg.pose.pose.orientation.z = 0
        odom_msg.pose.pose.orientation.w = 0

        odom_msg.pose.covariance = [cov_x, 0, 0, 0, 0, 0,
                                    0, cov_y, 0, 0, 0, 0,
                                    0, 0, cov_z, 0, 0, 0,
                                    0, 0, 0, 999999, 0, 0,
                                    0, 0, 0, 0, 999999, 0,
                                    0, 0, 0, 0, 0, 999999]

        self.ekf_vo_publisher.publish(odom_msg)

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

    roscomp.init("ekf_translation", args=args)
    try:
        node = EKFTranslation()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
