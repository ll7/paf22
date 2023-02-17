#!/usr/bin/env python
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from rospy import Publisher
from std_msgs.msg import Float32


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

        self.velocity_pub: Publisher = self.new_publisher(
            Float32,
            f"/carla/{self.role_name}/max_velocity",
            qos_profile=1)
        self.velocity = 4
        self.delta_velocity = 0.125
        self.max_velocity = 5
        self.min_velocity = 4
        self.__dv = self.delta_velocity

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
            # self.loginfo('Published dummy velocity: ' + str(self.velocity))
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
