#!/usr/bin/env python
from random import random
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
from rospy import Publisher, Subscriber
from ros_compatibility.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, Float32


# todo: docs
class EmergencyTriggerDummy(CompatibleNode):
    def __init__(self):
        super(EmergencyTriggerDummy, self).__init__('Emergency_trigger_dummy')
        self.loginfo('EmergencyTriggerDummy node started')

        self.control_loop_rate = self.get_param('control_loop_rate', 10)
        self.role_name = self.get_param('role_name', 'ego_vehicle')

        self.emergency_pub: Publisher = self.new_publisher(
            Bool,
            f"/carla/{self.role_name}/emergency",
            qos_profile=10
        )

    def run(self):
        """
        Starts the main loop of the node
        :return:
        """
        self.loginfo('EmergencyTriggerDummy node running')

        def loop(timer_event=None):
            """
            Main loop of the acting node
            :param timer_event: Timer event from ROS
            :return:
            """
            self.loginfo('EmergencyTriggerDummy node triggering emergency')
            self.emergency_pub.publish(True)

        self.new_timer(self.control_loop_rate, loop)
        self.spin()


def main(args=None):
    """
      main function starts the acting node
      :param args:
    """
    roscomp.init('emergency_trigger_dummy', args=args)

    try:
        node = EmergencyTriggerDummy()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == '__main__':
    main()
