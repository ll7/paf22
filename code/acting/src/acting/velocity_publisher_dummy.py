#!/usr/bin/env python
import math

import ros_compatibility as roscomp
import rospy
from carla_msgs.msg import CarlaSpeedometer
from geometry_msgs.msg import PoseStamped, Point
from ros_compatibility.node import CompatibleNode
from rospy import Publisher, Subscriber
from std_msgs.msg import Float32
from nav_msgs.msg import Path
from helper_functions import vector_angle

from trajectory_interpolation import points_to_vector


def dist_between_points(a: PoseStamped, b: PoseStamped) -> float:
    a_x = a.pose.position.x
    a_y = a.pose.position.y
    b_x = b.pose.position.x
    b_y = b.pose.position.y
    return math.sqrt((b_x - a_x) ** 2 + (b_y - a_y) ** 2)


PARKING_V: float = 2.0
PARKING_DUR: float = 15.0


class VelocityPublisherDummy(CompatibleNode):
    """
    This node publishes velocities. It can be used for testing.
    Published velocities move up and down in steps of delta_velocity between
    min_velocity and max_velocity.
    """
    def __init__(self):
        super(VelocityPublisherDummy, self).\
            __init__('velocity_publisher_dummy')

        self.control_loop_rate = self.get_param('control_loop_rate', 10)
        self.role_name = self.get_param('role_name', 'ego_vehicle')
        self.enabled = self.get_param('enabled', False)

        self.velocity_sub: Subscriber = self.new_subscription(
            CarlaSpeedometer,
            f"/carla/{self.role_name}/Speed",
            self.__set_velocity,
            qos_profile=1)

        self.velocity_pub: Publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/max_velocity",
            qos_profile=1)
        self.position_sub: Subscriber = self.new_subscription(
            Path,
            f"/paf/{self.role_name}/trajectory",
            self.__set_path,
            qos_profile=1)

        self.path_sub: Subscriber = self.new_subscription(
            PoseStamped,
            f"/paf/{self.role_name}/current_pos",
            self.__set_position,
            qos_profile=1)

        self.heading_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/current_heading",
            self.__set_heading,
            qos_profile=1
        )

        self.__cur_v: float = 0.0
        self.__is_moving: bool = False
        self.__is_parking: bool = True
        self.__start_time = None
        self.__current_point_id_id = -1
        self.__path = None
        self.__position: (float, float) = None  # x, y

        self.velocity = 3.0
        self.delta_velocity = 0.05
        self.max_velocity = 7.0
        self.min_velocity = 3.0

        self.__dv = self.delta_velocity

    def __set_velocity(self, data: CarlaSpeedometer) -> None:
        new_v = data.speed
        if not self.__is_moving and new_v > 0.125:
            # vehicle starts moving
            self.__is_moving = True
            self.__start_time = rospy.get_time()
        self.__cur_v = new_v

    def run(self):
        """
        Starts the main loop of the node
        :return:
        """
        if not self.enabled:
            return
        self.loginfo('VelocityPublisherDummy node running')

        def loop(timer_event=None):
            """
            Publishes dummy velocity
            :param timer_event: Timer event from ROS
            :return:
            """
            elapsed_time = 0
            if self.__start_time is not None:
                # vehicle has not started moving yet
                elapsed_time = rospy.get_time() - self.__start_time

            if elapsed_time > PARKING_DUR:
                # vehicle should have left the parking spot by now
                self.__is_parking = False

            if self.__is_parking:
                # Parking state
                self.velocity_pub.publish(PARKING_V)
            else:
                # Normal state
                # self.loginfo('Published velocity: ' + str(self.velocity))
                self.velocity_pub.publish(self.velocity)
                if self.velocity > self.max_velocity:
                    self.__dv = -self.delta_velocity
                if self.velocity < self.min_velocity:
                    self.__dv = self.delta_velocity
                self.velocity += self.__dv
        self.new_timer(self.control_loop_rate, loop)
        self.spin()


def main(args=None):
    """
    Main function starts the node
    :param args:
    """
    roscomp.init('velocity_publisher_dummy', args=args)

    try:
        node = VelocityPublisherDummy()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == '__main__':
    main()
