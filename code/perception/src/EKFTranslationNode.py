#!/usr/bin/env python

"""
This node publishes all relevant topics for the ekf node.
"""
import numpy as np
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from coordinate_transformation import CoordinateTransformer, GeoRef
from scipy.spatial.transform import Rotation as R


GPS_RUNNING_AVG_ARGS = 5  # points taken into account for the avg


class EKFTranslation(CompatibleNode):
    """
    Translates to the topic required by robot_pose_ekf.
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

        self.avg_xyz = np.zeros((GPS_RUNNING_AVG_ARGS, 3))
        self.__publish_counter: int = 0
        # 3D Odometry (GPS)
        self.ekf_vo_publisher = self.new_publisher(
            Odometry,
            "/vo",
            qos_profile=1)

    def update_imu_data(self, data: Imu):
        imu_data = Imu()

        imu_data.header.stamp = data.header.stamp
        imu_data.header.frame_id = "hero"

        imu_data.orientation.x = data.orientation.x
        imu_data.orientation.y = data.orientation.y
        imu_data.orientation.z = data.orientation.z
        imu_data.orientation.w = data.orientation.w
        imu_data.orientation_covariance = [0, 0, 0,
                                           0, 0, 0,
                                           0, 0, 0]

        # Rotationsmatrix aufstellen
        r = R.from_quat([imu_data.orientation.x,
                         imu_data.orientation.y,
                         imu_data.orientation.z,
                         imu_data.orientation.w])
        r = r.inv()
        rot_mat = r.as_matrix()

        # Werte für Linearbeschleunigung auslesen und umrechnen
        lin_x = data.linear_acceleration.x
        lin_y = data.angular_velocity.y
        lin_z = data.angular_velocity.z

        lin_v = np.array([[lin_x],
                          [lin_y],
                          [lin_z]])

        lin_res_v = np.matmul(rot_mat, lin_v)

        imu_data.linear_acceleration.x = lin_res_v.item(0)
        imu_data.linear_acceleration.y = lin_res_v.item(1)
        imu_data.linear_acceleration.z = lin_res_v.item(2)
        imu_data.linear_acceleration_covariance = [0.001, 0,     0,
                                                   0,     0.001, 0,
                                                   0,     0,     0.015]

        # Werte für Winkelbeschleunigung auslesen und umrechnen
        ang_x = data.angular_velocity.x
        ang_y = data.angular_velocity.y
        ang_z = data.angular_velocity.z

        ang_v = np.array([[ang_x],
                          [ang_y],
                          [ang_z]])
        ang_res_v = np.matmul(rot_mat, ang_v)

        imu_data.angular_velocity.x = ang_res_v.item(0)
        imu_data.angular_velocity.y = ang_res_v.item(1)
        imu_data.angular_velocity.z = ang_res_v.item(2)
        imu_data.angular_velocity_covariance = [0.001, 0,     0,
                                                0,     0.001, 0,
                                                0,     0,     0.001]

        self.ekf_imu_publisher.publish(imu_data)

    def update_gps_data(self, data: NavSatFix):
        # transform new measurement to global xyz-frame
        lat = data.latitude
        lon = data.longitude
        alt = data.altitude
        x, y, z = self.transformer.gnss_to_xyz(lat, lon, alt)

        # rotate avg matrix by one and add new element to the back
        self.avg_xyz = np.roll(self.avg_xyz, -1, axis=0)
        self.avg_xyz[-1] = np.matrix([x, y, z])

        # compute average of each column
        avg_x, avg_y, avg_z = np.mean(self.avg_xyz, axis=0)

        odom_msg = Odometry()

        odom_msg.header.stamp = data.header.stamp
        odom_msg.header.frame_id = "global"

        # Covariance todo: needs tweaking
        cov_x = 0.01
        cov_y = 0.01
        cov_z = 0.01

        odom_msg.pose.pose.position.x = avg_x
        odom_msg.pose.pose.position.y = avg_y
        odom_msg.pose.pose.position.z = avg_z

        odom_msg.pose.pose.orientation.x = 0
        odom_msg.pose.pose.orientation.y = 0
        odom_msg.pose.pose.orientation.z = 1
        odom_msg.pose.pose.orientation.w = 0

        odom_msg.pose.covariance = [cov_x, 0, 0, 0, 0, 0,
                                    0, cov_y, 0, 0, 0, 0,
                                    0, 0, cov_z, 0, 0, 0,
                                    0, 0, 0, 999999, 0, 0,
                                    0, 0, 0, 0, 999999, 0,
                                    0, 0, 0, 0, 0, 999999]

        self.__publish_counter += 1
        if self.__publish_counter % 5 == 0:
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
