#!/usr/bin/env python
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode


class Acting(CompatibleNode):
    def __init__(self):
        super(Acting, self).__init__('acting')
        self.loginfo('Acting node started')

        self.control_loop_rate = self.get_param('control_loop_rate', 10)

    def run(self):
        self.loginfo('Acting node running')

        def loop(timer_event=None):
            self.loginfo('Acting node loop')

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
