#!/usr/bin/env python
from math import atan, sin

import numpy
import ros_compatibility as roscomp
from carla_msgs.msg import CarlaSpeedometer
from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import Path
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile, DurabilityPolicy
from rospy import Publisher, Subscriber
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32, Bool
from tf.transformations import euler_from_quaternion


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
            f"/carla/{self.role_name}/path",
            self.__set_path,
            qos_profile=1)

        self.heading_sub: Subscriber = self.new_subscription(
            Imu,
            f"/carla/{self.role_name}/IMU",
            self.__set_heading,
            qos_profile=1)

        self.path_sub: Subscriber = self.new_subscription(
            NavSatFix,
            f"/carla/{self.role_name}/GPS",
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

    def __set_position(self, data: NavSatFix):
        self.__position = (data.latitude, data.longitude)

    def __set_path(self, data: Path):
        self.__path = data

    def __set_heading(self, data: Imu):  # todo: test
        rot_euler = euler_from_quaternion(data.orientation)
        self.__heading = rot_euler[2]

    def __set_velocity(self, data):
        self.__velocity = data.speed

    def __calculate_steer(self) -> float:
        L = 2  # dist between front and rear wheels todo: measure
        K = 10  # todo: tune
        alpha = self.__get_heading_error()
        look_ahead_dist = K * self.__velocity
        steering_angle = atan((2 * L * sin(alpha)) / look_ahead_dist)
        return steering_angle

    def __get_heading_error(self) -> float:
        my_heading = self.__heading
        target_rot = self.__get_closest_point_on_path().orientation
        rot_euler = euler_from_quaternion(target_rot)
        target_heading = rot_euler[2]
        return target_heading - my_heading

    def __get_closest_point_on_path(self) -> Pose:
        min_dist: float = 10e1000
        min_pos: Pose = None
        for pose in self.__path.poses:
            if self.__dist_to(pose.position) <= min_dist:
                min_dist = self.__dist_to(pose.position)
                min_pos = pose
        if min_pos is None:
            raise Exception()
        return min_pos

    def __dist_to(self, pos: Point) -> float:
        my_pos = numpy.array((self.__position[0], self.__position[1], pos[2]))
        return numpy.linalg.norm(my_pos - pos)


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
