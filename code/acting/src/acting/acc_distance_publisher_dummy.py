#!/usr/bin/env python
from random import random

import ros_compatibility as roscomp
from carla_msgs.msg import CarlaSpeedometer
from ros_compatibility.node import CompatibleNode
from rospy import Publisher, Subscriber
from std_msgs.msg import Float32


class AccDistancePublisherDummy(CompatibleNode):
    """
    This node publishes velocities. It can be used for testing.
    Published velocities move up and down in steps of delta_velocity between
    min_velocity and max_velocity.
    """

    def __init__(self):
        super(AccDistancePublisherDummy, self). \
            __init__('acc_distance_publisher_dummy')

        self.control_loop_rate = 10
        self.role_name = self.get_param('role_name', 'ego_vehicle')
        self.enabled = self.get_param('enabled', False)

        self.distance_pub: Publisher = self.new_publisher(
            Float32,
            f"/carla/{self.role_name}/acc_distance",
            qos_profile=1)

        self.speed_sub: Subscriber = self.new_subscription(
            CarlaSpeedometer,
            f"/carla/{self.role_name}/Speed",
            self.__set_speed,
            qos_profile=1)

        self.velocity_car_ahead = 10  # = 36 km/h
        self.distance = 36
        self.velocity = 0

    def run(self):
        """
        Starts the main loop of the node
        :return:
        """
        if not self.enabled:
            return
        self.loginfo('AccDistancePublisherDummy node running')

        def loop(timer_event=None):
            """
            Publishes dummy velocity
            :param timer_event: Timer event from ROS
            :return:
            """

            max_delta_v = 1
            self.velocity_car_ahead = \
                self.velocity_car_ahead + \
                random.uniform(-max_delta_v, max_delta_v)
            self.distance = \
                self.distance + \
                self.velocity_car_ahead * (1 / self.control_loop_rate)
            self.distance = \
                self.distance - \
                self.velocity * (1 / self.control_loop_rate)
            self.distance_pub.publish(self.distance)

            self.new_timer(self.control_loop_rate, loop)

        self.spin()

    def __set_speed(self, data: CarlaSpeedometer):
        self.velocity = data.speed


def main(args=None):
    """
    Main function starts the node
    :param args:
    """
    roscomp.init('acc_distance_publisher_dummy', args=args)

    try:
        node = AccDistancePublisherDummy()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == '__main__':
    main()
