#!/usr/bin/env python
import math
from math import atan, sin

import numpy
import ros_compatibility as roscomp
from carla_msgs.msg import CarlaSpeedometer
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Path
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile, DurabilityPolicy
from rospy import Publisher, Subscriber
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Bool
from tf.transformations import euler_from_quaternion

from helper_functions import quaternion2heading


# todo: docs
# todo: test
class PurePursuitController(CompatibleNode):
    def __init__(self):
        super(PurePursuitController, self).__init__('pure_pursuit_controller')
        self.loginfo('PurePursuitController node started')

        self.control_loop_rate = self.get_param('control_loop_rate', 0.05)
        self.role_name = self.get_param('role_name', 'ego_vehicle')

        self.position_sub: Subscriber = self.new_subscription(
            Path,
            f"/carla/{self.role_name}/trajectory",
            self.__set_path,
            qos_profile=1)

        self.heading_sub: Subscriber = self.new_subscription(
            Imu,
            f"/carla/{self.role_name}/IMU",
            self.__set_heading,
            qos_profile=1)

        self.path_sub: Subscriber = self.new_subscription(
            PoseStamped,
            f"/carla/{self.role_name}/current_pos",
            self.__set_position,
            qos_profile=1)

        self.velocity_sub: Subscriber = self.new_subscription(
            CarlaSpeedometer,
            f"/carla/{self.role_name}/Speed",
            self.__set_velocity,
            qos_profile=1)

        self.pure_pursuit_steer_pub: Publisher = self.new_publisher(
            Float32,
            f"/carla/{self.role_name}/pure_pursuit_steer",
            qos_profile=1)

        self.status_pub = self.new_publisher(  # todo: delete (this is only
            # necessary if vehicle controller isn't running)
            Bool, f"/carla/{self.role_name}/status",
            qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        self.__position: (float, float) = None  # latitude, longitude in deg
        self.__path: Path = None
        self.__heading: float = None
        self.__velocity: float = None

    def run(self):
        """
        Starts the main loop of the node
        :return:
        """
        self.loginfo('PurePursuitController node running')
        self.status_pub.publish(True)  # todo: delete (this is only

        # necessary if vehicle controller isn't running)

        def loop(timer_event=None):
            """
            Main loop of the acting node
            :param timer_event: Timer event from ROS
            :return:
            """
            if self.__path is None:
                self.logerr("PurePursuitController hasn't received a path yet "
                            "and can therefore not publish steering")
                return
            if self.__position is None:
                self.logerr("PurePursuitController hasn't received the"
                            "position of the vehicle yet "
                            "and can therefore not publish steering")
                return

            if self.__heading is None:
                self.logerr("PurePursuitController hasn't received the heading"
                            "of the vehicle yet and can therefore "
                            "not publish steering")
                return

            if self.__velocity is None:
                self.logerr("PurePursuitController hasn't received the "
                            "velocity of the vehicle yet "
                            "and can therefore not publish steering")
                return
            self.pure_pursuit_steer_pub.publish(self.__calculate_steer())

        self.new_timer(self.control_loop_rate, loop)
        self.spin()

    def __set_position(self, data: PoseStamped):
        x = data.pose.position.x
        y = data.pose.position.y
        self.__position = (x, y)

    def __set_path(self, data: Path):
        self.__path = data

    def __set_heading(self, data: Imu):  # todo: test
        self.__heading = quaternion2heading(
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w)

    def __set_velocity(self, data):
        self.__velocity = data.speed

    def __calculate_steer(self) -> float:
        L = 2  # dist between front and rear wheels todo: measure
        K = 10  # todo: tune
        alpha = self.__get_heading_error()
        self.loginfo(f"Heading error: {alpha}")
        # offset speed +1 km/h to avoid division by 0
        look_ahead_dist = K * (self.__velocity + 0.2777)
        steering_angle = atan((2 * L * sin(alpha)) / look_ahead_dist)
        return steering_angle

    def __get_heading_error(self) -> float:
        # todo: the calculation of the angle could be done
        #  before every point before publishing a path
        my_heading = self.__heading
        target_idx = self.__get_closest_point_on_path_index()
        target_point: PoseStamped
        target_point = self.__path.poses[target_idx]

        # todo: move this calculation to a separate function and use it to
        #  calculate w for every point on the path.
        #  __get_omega(Pose_-1, Pose, Pose_+1) -> check for identical points...

        # get the previous and next point on the path
        path_len = len(self.__path.poses)
        tp_after: PoseStamped
        tp_after = self.__path.poses[min(path_len, target_idx + 1)]
        tp_prev: PoseStamped
        tp_prev = self.__path.poses[max(0, target_idx - 1)]

        # calculate the average direction of the two vectors
        # v1 = tp_before -> tp
        # v2 = tp -> tp_after
        avg_heading = 0.0
        avg_heading_args = 0
        tp_x = target_point.pose.position.x
        tp_y = target_point.pose.position.y
        if tp_prev is not target_point:
            tp_prev_x = tp_prev.pose.position.x
            tp_prev_y = tp_prev.pose.position.y
            avg_heading += self.__get_vector_direction(tp_prev_x, tp_prev_y,
                                                       tp_x, tp_y)
            avg_heading_args += 1
        if tp_after is not target_point:
            tp_after_x = tp_after.pose.position.x
            tp_after_y = tp_after.pose.position.y
            avg_heading += self.__get_vector_direction(tp_x, tp_y,
                                                       tp_after_x, tp_after_y)
            avg_heading_args += 1
        if avg_heading_args is 0:
            raise Exception()

        target_heading = avg_heading / avg_heading_args
        return target_heading - my_heading

    def __get_vector_direction(self, x1, y1, x2, y2) -> float:
        theta = math.atan(x2 - x1 / y2 - y1)
        return math.degrees(theta)

    def __get_closest_point_on_path_index(self) -> int:
        min_dist: float = 10e1000
        min_pos_index = -1
        for i in range(0, len(self.__path.poses)):
            pose: PoseStamped
            pose = self.__path.poses[i]
            if self.__dist_to(pose.pose.position) <= min_dist:
                min_dist = self.__dist_to(pose.pose.position)
                min_pos_index = i
        if min_pos_index == -1:
            raise Exception()
        return min_pos_index

    def __get_closest_point_on_path(self) -> Pose:
        min_dist: float = 10e1000
        min_pos: Pose = None
        for pose in self.__path.poses:
            if self.__dist_to(pose.pose.position) <= min_dist:
                min_dist = self.__dist_to(pose.pose.position)
                min_pos = pose.pose
        if min_pos is None:
            raise Exception()
        return min_pos

    def __dist_to(self, pos: Point) -> float:
        """
        Distance between current position and target position (only (x,y))
        :param pos: targeted position
        :return: distance
        """
        x_current = self.__position[0]
        y_current = self.__position[1]
        x_target = pos.x
        y_target = pos.y
        d = (x_target - x_current)**2 + (y_target - y_current)**2
        return math.sqrt(d)


def main(args=None):
    """
      main function starts the acting node
      :param args:
    """
    roscomp.init('pure_pursuit_controller', args=args)

    try:
        node = PurePursuitController()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == '__main__':
    main()
