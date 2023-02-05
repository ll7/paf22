#!/usr/bin/env python

"""
This node publishes all relevant topics for the ekf node.
"""
import math

import numpy as np
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistWithCovariance  # , Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from coordinate_transformation import CoordinateTransformer, GeoRef
from tf.transformations import euler_from_quaternion
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
        self.__cur_imu: Imu = Imu()
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

        self.__cur_v: float = 0
        self.velocity_subscriber = self.new_subscription(
            Float32,
            "/carla/" + self.role_name + "/velocity_as_float",
            self.update_velocity,
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

        self.__heading: float = 0
        self.__heading_publisher = self.new_publisher(
            Float32,
            f"/carla/{self.role_name}/current_heading",
            qos_profile=1)

    def update_imu_data(self, data: Imu):
        # Take measurements from IMU and republish with covariance
        self.__cur_imu = data
        imu_data = Imu()

        imu_data.header.stamp = data.header.stamp
        imu_data.header.frame_id = "hero"

        imu_data.orientation = data.orientation
        imu_data.orientation_covariance = [0, 0, 0,
                                           0, 0, 0,
                                           0, 0, 0]

        imu_data.linear_acceleration = data.linear_acceleration
        imu_data.linear_acceleration_covariance = [0.001, 0,     0,
                                                   0,     0.001, 0,
                                                   0,     0,     0.015]

        imu_data.angular_velocity = data.angular_velocity
        imu_data.angular_velocity_covariance = [0.001, 0,     0,
                                                0,     0.001, 0,
                                                0,     0,     0.001]

        self.ekf_imu_publisher.publish(imu_data)

        # Calculate the heading based on the orientation given by the IMU
        data_orientation_q = [data.orientation.x,
                              data.orientation.y,
                              data.orientation.z,
                              data.orientation.w]

        roll, pitch, yaw = euler_from_quaternion(data_orientation_q)
        raw_heading = math.atan2(roll, pitch)

        # transform raw_heading so that:
        # ---------------------------------------------------------------
        # | 0 = x-axis | pi/2 = y-axis | pi = -x-axis | -pi/2 = -y-axis |
        # ---------------------------------------------------------------
        heading = (raw_heading - (math.pi / 2)) % (2 * math.pi) - math.pi
        self.__heading = heading
        self.__heading_publisher.publish(self.__heading)

    def update_velocity(self, data: Float32):
        self.__cur_v = data.data

    def compute_twist(self) -> TwistWithCovariance:
        imu_orientation = self.__cur_imu.orientation
        imu_q = [imu_orientation.x,
                 imu_orientation.y,
                 imu_orientation.z,
                 imu_orientation.w]
        roll, pitch, yaw = euler_from_quaternion(imu_q)
        v = self.__cur_v

        angle = math.atan2(roll, pitch)
        hor_heading = (angle - (math.pi / 2)) % (2 * math.pi) - math.pi
        ver_heading = math.atan2(roll, yaw)  # ???
        v_x = math.cos(hor_heading) * math.sin(ver_heading) * v
        v_y = math.sin(hor_heading) * math.sin(ver_heading) * v
        v_z = math.cos(ver_heading) * v

        # build Twist msg
        res = TwistWithCovariance()

        # ------------------------------------
        # Test with rotated velocity vector ->
        r = R.from_quat(imu_q)
        v_v = np.array([v_x, v_y, v_z])
        rot_mat = r.as_matrix()
        v_rotated = np.matmul(rot_mat, v_v)

        res.twist.linear.x = v_rotated[0]
        res.twist.linear.y = v_rotated[1]
        res.twist.linear.z = v_rotated[2]
        # <-
        # ------------------------------------

        # ------------------------------------
        # Uncomment to use velocity vector without rotation ->
        # res.twist.linear.x = v_x
        # res.twist.linear.y = v_y
        # res.twist.linear.z = v_z
        # <-
        # ------------------------------------

        # angular v = 0 for now
        res.twist.angular.x = 0
        res.twist.angular.y = 0
        res.twist.angular.z = 0

        res.covariance = [1, 0, 0, 0, 0, 0,
                          0, 1, 0, 0, 0, 0,
                          0, 0, 1, 0, 0, 0,
                          0, 0, 0, 999999, 0, 0,
                          0, 0, 0, 0, 999999, 0,
                          0, 0, 0, 0, 0, 999999]

        # debugging ->
        v_sum = math.sqrt(v_x**2 + v_y**2 + v_z**2)
        self.loginfo(f"V: {round(v, 3)} \t"
                     f"V_sum: {round(v_sum, 3)} \t"
                     f"Hor: {round(math.degrees(hor_heading), 1)} \t"
                     f"Vrt: {round(math.degrees(ver_heading), 1)} \t"
                     f"V_X: {round(v_x, 3)} \t"
                     f"rV_X: {round(v_rotated[0], 3)} \t"
                     f"V_Y: {round(v_y, 3)} \t"
                     f"rV_Y: {round(v_rotated[1], 3)} \t"
                     f"V_Z: {round(v_z, 3)} \t"
                     f"rV_Z: {round(v_rotated[2], 3)}")
        # <-

        return res

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

        # Covariance for pose
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

        odom_msg.twist = self.compute_twist()
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
