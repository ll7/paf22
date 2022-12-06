#!/usr/bin/env python
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from rospy import Publisher
from std_msgs.msg import Float32


# todo: docs
class VelocityPublisherDummy(CompatibleNode):
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
        self.velocity = 10
        self.delta_velocity = 0.25

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
            Main loop of the acting node
            :param timer_event: Timer event from ROS
            :return:
            """
            self.loginfo('Published dummy velocity: ' + str(self.velocity))
            self.velocity_pub.publish(self.velocity)
            if self.velocity > 25:
                self.delta_velocity = -0.25
            if self.velocity < 5:
                self.delta_velocity = 0.25
            self.velocity += self.delta_velocity
        self.new_timer(self.control_loop_rate, loop)
        self.spin()


def main(args=None):
    """
      main function starts the acting node
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
