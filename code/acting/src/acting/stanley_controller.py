#!/usr/bin/env python
# from typing import List

from math import atan, sqrt, radians
# import numpy as np
import ros_compatibility as roscomp
from carla_msgs.msg import CarlaSpeedometer
from geometry_msgs.msg import PoseStamped, Point  # , Pose, Quaternion
from nav_msgs.msg import Path
from ros_compatibility.node import CompatibleNode
# from ros_compatibility.qos import QoSProfile, DurabilityPolicy
from rospy import Publisher, Subscriber
# from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32  # Bool
from acting.msg import Debug

from helper_functions import vectors_to_angle
from trajectory_interpolation import points_to_vector


# todo: docs
# todo: test
class StanleyController(CompatibleNode):
    def __init__(self):
        super(StanleyController, self).__init__('stanley_controller')
        self.loginfo('StanleyController node started')

        self.control_loop_rate = self.get_param('control_loop_rate', 0.05)
        self.role_name = self.get_param('role_name', 'ego_vehicle')

        # Subscriber
        self.position_sub: Subscriber = self.new_subscription(
            Path,
            f"/carla/{self.role_name}/trajectory",
            self.__set_path,
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

        # self.heading_sub: Subscriber = self.new_subscription(
        #    Float32,
        #    f"/carla/{self.role_name}/current_heading",
        #    self.__set_heading,
        #    qos_profile=1)

        # Publisher
        self.stanley_steer_pub: Publisher = self.new_publisher(
            Float32,
            f"/carla/{self.role_name}/stanley_steer",
            qos_profile=1)

        self.debug_publisher: Publisher = self.new_publisher(
            Debug,
            f"/carla/{self.role_name}/debug",
            qos_profile=1)

        self.__position: (float, float) = None  # x, y
        self.__last_pos: (float, float) = None
        self.__path: Path = None
        self.__heading: float = None
        self.__velocity: float = None
        self.__tp_idx: int = 0  # target waypoint index
        # error when there are no targets

    def run(self):
        """
        Starts the main loop of the node
        :return:
        """
        self.loginfo('StanleyController node running')

        def loop(timer_event=None):
            """
            Main loop of the acting node
            :param timer_event: Timer event from ROS
            :return:
            """
            if self.__path is None:
                self.logwarn("StanleyController hasn't received a path yet"
                             "and can therefore not publish steering")
                return
            if self.__position is None:
                self.logwarn("StanleyController hasn't received the"
                             "position of the vehicle yet "
                             "and can therefore not publish steering")
                return

            if self.__heading is None:
                self.logwarn("StanleyController hasn't received the"
                             "heading of the vehicle yet and"
                             "can therefore not publish steering")
                return

            if self.__velocity is None:
                self.logwarn("StanleyController hasn't received the "
                             "velocity of the vehicle yet "
                             "and can therefore not publish steering")
                return
            self.stanley_steer_pub.publish(self.__calculate_steer())

        self.new_timer(self.control_loop_rate, loop)
        self.spin()

    def __set_position(self, data: PoseStamped, min_diff=0.05):
        """
        Updates the current position of the vehicle
        To avoid problems when the car is stationary, new positions will only
        be accepted, if they are a certain distance from the current one
        :param data: new position as PoseStamped
        :param min_diff: minium difference between new and current point for
        the new point to be accepted
        :return:
        """
        if self.__position is None:
            x0 = data.pose.position.x
            y0 = data.pose.position.y
            self.__position = (x0, y0)
            return

        # check if the new position is valid
        dist = self.__dist_to(data.pose.position)
        if dist < min_diff:
            # for debugging purposes:
            self.logwarn("New position disregarded, "
                         f"as dist ({round(dist, 3)}) to current pos "
                         f"< min_diff ({round(min_diff, 3)})")
            return

        old_x = self.__position[0]
        old_y = self.__position[1]
        self.__last_pos = (old_x, old_y)
        new_x = data.pose.position.x
        new_y = data.pose.position.y
        self.__position = (new_x, new_y)
        # todo: delete when using carla/hero/current_heading
        self.__set_heading()

    def __set_path(self, data: Path):
        self.__path = data

# --------------------------------------------------------------------
# -> delete / uncomment this block, when carla/hero/current_heading is accurate
# --------------------------------------------------------------------

    def __set_heading(self):
        """
        Updates the current heading
        :return:
        """
        if self.__position is None:
            self.logwarn("__get_heading: Current Position not set")
            self.__heading = 0
            return
        if self.__last_pos is None:
            self.logwarn("__get_heading: Last Position not set")
            self.__heading = 0
            return

        cur_x, cur_y = points_to_vector(
                                        (self.__last_pos[0],
                                         self.__last_pos[1]),
                                        (self.__position[0],
                                         self.__position[1])
                                        )
        # maybe remove weight if it doesn't help (after fixing gps signal)
        # code without weight:
        # self.__heading = vectors_to_angle(cur_x, cur_y, 1, 0)
        # ->
        if self.__heading is not None:
            old_heading: float = self.__heading
            new_heading: float = vectors_to_angle(cur_x, cur_y, 1, 0)
            self.__heading = (2 * new_heading + 1 * old_heading) / 3
        else:
            self.__heading = vectors_to_angle(cur_x, cur_y, 1, 0)
        # <-

#   def __set_heading(self, data: Float32):
#        self.__heading = data.data

# --------------------------------------------------------------------
# <-
# --------------------------------------------------------------------

    def __set_velocity(self, data: CarlaSpeedometer):
        self.__velocity = data.speed

    def __calculate_steer(self) -> float:
        """
        Calculates the steering angle based on the current information
        :return:
        """
        k_ce = 3.0  # todo: tune

        current_velocity: float
        if self.__velocity <= 1:
            current_velocity = 1
        else:
            current_velocity = self.__velocity

        closest_point_idx = self.__get_closest_point_index()
        closest_point: PoseStamped = self.__path.poses[closest_point_idx]

        cross_err = self.__dist_to(closest_point.pose.position)

        traj_heading = self.__get_path_heading(closest_point_idx)
        heading_err = radians(self.__heading) - traj_heading

        steering_angle = heading_err + atan((k_ce * cross_err) /
                                            current_velocity)

        self.loginfo(f"C_Err: {round(cross_err, 4)} \t"
                     f"H_Err: {round(heading_err, 4)} \t"
                     f"steering_angle: {round(steering_angle, 3)}")

        return steering_angle

    def __get_closest_point_index(self) -> int:
        if len(self.__path.poses) < 2:
            return -1

        min_dist = 10e100
        min_dist_idx = -1

        for i in range(0, len(self.__path.poses)):
            temp_pose: PoseStamped = self.__path.poses[i]
            dist = self.__dist_to(temp_pose.pose.position)
            if min_dist > dist:
                min_dist = dist
                min_dist_idx = i
        return min_dist_idx

    def __get_path_heading(self, index: int) -> float:
        """
        Calculates the heading of the current path at index
        :param index: point of interest
        :return: heading at path[index]
        """
        cur_pos: Point = self.__path.poses[index].pose.position
        l_path = len(self.__path.poses)

        if l_path == 1:
            return 0

        heading_sum = 0
        heading_sum_args = 0

        if index > 0:
            # Calculate heading from the previous point on the trajectory
            prv_heading = 0
            prv_point: Point = self.__path.poses[index-1].pose.position

            prv_xe = self.__get_x_vector(prv_point)
            prv_xev_x, prv_xev_y = points_to_vector((prv_point.x, prv_point.y),
                                                    (prv_xe.x, prv_xe.y))
            prv_v_x, prv_v_y = points_to_vector((prv_point.x, prv_point.y),
                                                (cur_pos.x, cur_pos.y))

            prv_heading = vectors_to_angle(prv_xev_x, prv_xev_y,
                                           prv_v_x, prv_v_y)
            heading_sum += radians(prv_heading)
            heading_sum_args += 1
        elif index < l_path - 1:
            # Calculate heading to the following point on the trajectory
            aft_heading = 0
            aft_point: Point = self.__path.poses[index + 1].pose.position
            aft_xe = self.__get_x_vector(aft_point)
            aft_xev_x, aft_xev_y = points_to_vector((aft_point.x, aft_point.y),
                                                    (aft_xe.x, aft_xe.y))
            aft_v_x, aft_v_y = points_to_vector((aft_point.x, aft_point.y),
                                                (cur_pos.x, cur_pos.y))

            aft_heading = vectors_to_angle(aft_xev_x, aft_xev_y,
                                           aft_v_x, aft_v_y)
            heading_sum += radians(aft_heading)
            heading_sum_args += 1
        return heading_sum / heading_sum_args

    def __get_x_vector(self, point: Point) -> Point:
        x = point.x
        y = point.y
        res = Point(x+1, y, 0)
        return res

    def __get_target_point_index(self, ld: float) -> int:
        """
        Get the index of the target point on the current trajectory based on
        the look ahead distance.
        :param ld: look ahead distance
        :return:
        """
        if len(self.__path.poses) < 2:
            return -1

        min_dist = 10e1000
        min_dist_idx = -1
        # might be more elegant to only look at points
        # _ahead_ of the closest point on the trajectory
        for i in range(self.__tp_idx, len(self.__path.poses)):
            pose: PoseStamped = self.__path.poses[i]
            dist = self.__dist_to(pose.pose.position)
            dist2ld = dist - ld
            # can be optimized
            if min_dist > dist2ld > 0:
                min_dist = dist2ld
                min_dist_idx = i
        return min_dist_idx

    def __dist_to(self, pos: Point) -> float:
        """
        Distance between current position and target position (only (x,y))
        :param pos: targeted position
        :return: distance
        """
        x_cur = self.__position[0]
        y_cur = self.__position[1]
        x_target = pos.x
        y_target = pos.y
        d = (x_target - x_cur) ** 2 + (y_target - y_cur) ** 2
        return sqrt(d)


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
