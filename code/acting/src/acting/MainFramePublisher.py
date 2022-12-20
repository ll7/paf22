#!/usr/bin/env python
import ros_compatibility as roscomp
import rospy
from ros_compatibility.node import CompatibleNode
import tf


class MainFramePublisher(CompatibleNode):

    def __init__(self):
        super(MainFramePublisher, self).__init__('main_frame_publisher')
        self.loginfo('MainFramePublisher node started')

        self.control_loop_rate = self.get_param('control_loop_rate', 0.05)
        self.role_name = self.get_param('role_name', 'ego_vehicle')

    def run(self):
        self.loginfo('MainFramePublisher node running')
        br = tf.TransformBroadcaster()

        def loop(timer_event=None):
            br.sendTransform((-983.5, 5373.2, 0.0),  # todo: change to follow
                             # current pos
                             (0.0, 0.0, 0.0, 1.0),
                             rospy.Time.now(),
                             "main",
                             "hero",
                             )

        self.new_timer(self.control_loop_rate, loop)
        self.spin()


def main(args=None):
    """
    Main function starts the node
    :param args:
    """
    roscomp.init('main_frame_publisher', args=args)

    try:
        node = MainFramePublisher()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == '__main__':
    main()
