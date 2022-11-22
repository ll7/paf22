#!/usr/bin/env python
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from ackermann_msgs.msg import AckermannDrive
from rospy import Publisher


class Acting(CompatibleNode):
    def __init__(self):
        super(Acting, self).__init__('acting')
        self.loginfo('Acting node started')

        self.control_loop_rate = self.get_param('control_loop_rate', 0.1)
        self.role_name = self.get_param('role_name', 'ego_vehicle')

        self.ackermann_publisher: Publisher = self.new_publisher(
            AckermannDrive,
            f'/carla/{self.role_name}/ackermann_cmd',
            qos_profile=0
        )

    def run(self):
        self.loginfo('Acting node running')

        def loop(timer_event=None):
            self.loginfo('Acting node loop')
            message = AckermannDrive()
            message.speed = 1
            self.ackermann_publisher.publish(message)
            self.loginfo('Acting message published')

        self.new_timer(self.control_loop_rate, loop)
        self.spin()


def main(args=None):
    """
    main function runs the node
    :param args:
    """
    roscomp.init('acting', args=args)

    try:
        node = Acting()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == '__main__':
    main()
