#!/usr/bin/env python
from typing import List

import numpy as np
import ros_compatibility as roscomp
from carla_msgs.msg import CarlaSpeedometer
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile, DurabilityPolicy
from rospy import Publisher, Subscriber
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32, Bool

from helper_functions import calc_egocar_yaw, normalize_angle, calc_path_yaw, \
    quaternion2heading, heading2quaternion


# todo: docs
# todo: test
class StanleyController(CompatibleNode):
    def __init__(self):
        super(StanleyController, self).__init__('stanley_controller')
        self.loginfo('StanleyController node started')

        self.control_loop_rate = self.get_param('control_loop_rate', 0.05)
        self.role_name = self.get_param('role_name', 'ego_vehicle')
        self.k: float = 0.5  # Control gain
        self.Kp: float = 1.0  # Speed proportional gain
        self.L: float = 2.9  # [m] Wheel base of vehicle.
        self.max_steer: float = np.deg2rad(30.0)
        self.min_speed: float = 10  # Minimum speed used for calculation,
        # to avoid infinite values when standing still.

        self.heading_error: float = 0.0

        self._speed_to_change_k_factor: float = 70 / 3.6
        self._k_factor_change_at_high_speed: float = 2.0

        self.trajectory_sub: Subscriber = self.new_subscription(
            Path,
            "/carla/" + self.role_name + "/trajectory",
            self.__set_trajectory,
            qos_profile=1)

        self.heading_sub: Subscriber = self.new_subscription(
            Imu,
            f"/carla/{self.role_name}/IMU",
            self.__set_heading,
            qos_profile=1)

        self.position_sub: Subscriber = self.new_subscription(
            PoseStamped,
            f"/carla/{self.role_name}/current_pos",
            self.__set_position,
            qos_profile=1)

        self.velocity_sub: Subscriber = self.new_subscription(
            CarlaSpeedometer,
            f"/carla/{self.role_name}/Speed",
            self.__set_velocity,
            qos_profile=1)

        self.stanley_steer_pub: Publisher = self.new_publisher(
            Float32,
            f"/carla/{self.role_name}/stanley_steer",
            qos_profile=1)

        self.status_pub = self.new_publisher(  # todo: delete (this is only
            # necessary if vehicle controller isn't running)
            Bool, f"/carla/{self.role_name}/status",
            qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        self.__position: PoseStamped = None  # latitude, longitude in deg
        self.__path: Path = None
        self.__heading: float = None
        self.__velocity: float = None

    def run(self):
        """
        Starts the main loop of the node
        :return:
        """
        self.loginfo('StanleyController node running')
        self.status_pub.publish(True)  # todo: delete (this is only

        # necessary if vehicle controller isn't running)

        def loop(timer_event=None):
            """
            Main loop of the acting node
            :param timer_event: Timer event from ROS
            :return:
            """
            if self.__path is None:
                self.loginfo("StanleyController hasn't received a path yet "
                             "and can therefore not publish steering")
                return
            if self.__position is None:
                self.loginfo("StanleyController hasn't received the"
                             "position of the vehicle yet "
                             "and can therefore not publish steering")
                return

            if self.__heading is None:
                self.loginfo("StanleyController hasn't received the heading"
                             "of the vehicle yet and can therefore "
                             "not publish steering")
                return

            if self.__velocity is None:
                self.loginfo("StanleyController hasn't received the "
                             "velocity of the vehicle yet "
                             "and can therefore not publish steering")
                return
            self.stanley_steer_pub.publish(
                self.run_step(
                    self.__path,
                    self.__position,
                    self.__velocity,
                    False))

        self.new_timer(self.control_loop_rate, loop)
        self.spin()

    def __set_position(self, data: PoseStamped):
        position = data

        heading = self.__heading
        quat = heading2quaternion(heading)
        position.pose.orientation.x = quat[0]
        position.pose.orientation.y = quat[1]
        position.pose.orientation.z = quat[2]
        position.pose.orientation.w = quat[3]

        self.__position = position

    def __set_trajectory(self, data: Path):
        self.__path = data

    def __set_heading(self, data: Imu):  # todo: test
        self.__heading = quaternion2heading(
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w)

    def __set_velocity(self, data):
        self.__velocity = data.speed

    def run_step(self, msg: Path, pose: PoseStamped, speed: float,
                 is_reverse: bool) -> [float, float, float]:
        """
        Runs the Stanley-Controller calculations once
        Args:
            msg (Path): Path message to follow
            pose (PoseStamped): Pose of the Ego Vehicle
            speed (float): speed of Ego Vehicle
            is_reverse (bool): sets the stanley controller to steer backwards
        Returns:
           steering_angle [float]: Steering angle
           target_speed [float]: The speed the vehicle should drive
           distance [float]: distance to the point we want to drive to
        """
        # get the path points of the message
        path = msg.poses

        # get target point to drive to
        current_target_idx, error_front_axle, target_speed, distance = \
            self.calc_target_index(msg, pose, is_reverse)

        # compute heading error correction (adjusted if reversed)
        self.loginfo("Path length: ")
        self.loginfo(path.__len__())
        self.loginfo("current target index: ")
        self.loginfo(current_target_idx)
        theta_e: float = normalize_angle(
            calc_path_yaw(path, current_target_idx) + (
                calc_egocar_yaw(pose) if is_reverse else
                -calc_egocar_yaw(pose))
        )
        self.heading_error = theta_e

        # assures to not divide by too small numbers
        if abs(speed) < self.min_speed:
            speed: float = self.min_speed

        # adjust k parameter for steering stability at higher speeds
        if speed > self._speed_to_change_k_factor:
            k: float = self.k * self._k_factor_change_at_high_speed
        else:
            k: float = self.k

        # compute cross track error correction
        theta_d: float = np.arctan2(
            k * error_front_axle / max([1, 0.4 * speed]),
            speed)

        # compute steer
        delta: float = theta_e + theta_d

        # return the following: clip steering, the desired end speed,
        # distance to point that is looked at
        return np.clip(delta, -self.max_steer,
                       self.max_steer), target_speed, distance

    def calc_target_index(self, msg: Path, pose: PoseStamped,
                          is_reverse: bool) -> [
                          int, float, float, float]:
        """
            Calculates the index of the closest Point on the Path relative
            to the front axle
            Args:
                msg (Path): Path message to follow
                pose (PoseStamped): Pose of the Ego Vehicle
                is_reverse (bool): true if we drive backwards
            Returns:
                target_idx [int]: Index of target point
                error_front_axle [float]: Distance from front axle to
                                            target point
                target_speed [float]: The speed the vehicle should drive
                distance [float]: distance to the point we want to drive to
            """
        # get points of path to follow
        path = msg.poses

        # handle edge case if there is no path or no target speeds
        # if len(path) == 0 or len(msg.target_speed) == 0:
        #     return 0, 0, 0, 0

        # Calc front axle position
        yaw: float = calc_egocar_yaw(pose.pose)

        # calculate reference point for the stanley controller
        fx: float = 0.0
        fy: float = 0.0
        if is_reverse:
            fx = pose.pose.position.x - self.L * np.cos(yaw)
            fy = pose.pose.position.y - self.L * np.sin(yaw)
        else:
            fx = pose.pose.position.x + self.L * np.cos(yaw)
            fy = pose.pose.position.y + self.L * np.sin(yaw)

        # Search the nearest point index and distance to it

        px: List[float] = [pose.pose.position.x for pose in path]
        py: List[float] = [pose.pose.position.y for pose in path]
        dx: List[float] = [fx - icx for icx in px]
        dy: List[float] = [fy - icy for icy in py]
        d: np.ndarray = np.hypot(dx, dy)
        target_idx: int = np.argmin(d)
        distance: float = d[target_idx]

        # Project RMS error onto front axle vector
        front_axle_vec: List[float] = [-np.cos(yaw + np.pi / 2),
                                       -np.sin(yaw + np.pi / 2)]
        error_front_axle: float = np.dot([dx[target_idx], dy[target_idx]],
                                         front_axle_vec)

        # return:
        # point index on path to drive to
        # error of the front axle to disired path
        # the disired speed of the path that we look at
        # distance to the point we want to drive to
        return target_idx, error_front_axle, 0, distance


def main(args=None):
    """
    Main function starts the node
    :param args:
    """
    roscomp.init('stanley_controller', args=args)

    try:
        node = StanleyController()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == '__main__':
    main()
