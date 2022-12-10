#!/usr/bin/env python
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from carla_msgs.msg import CarlaEgoVehicleControl
from rospy import Publisher
from ros_compatibility.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool


class Acting(CompatibleNode):
    def __init__(self):
        super(Acting, self).__init__('acting')
        self.loginfo('Acting node started')

        self.control_loop_rate = self.get_param('control_loop_rate', 0.05)
        self.role_name = self.get_param('role_name', 'ego_vehicle')

        self.control_publisher: Publisher = self.new_publisher(
            CarlaEgoVehicleControl,
            f'/carla/{self.role_name}/vehicle_control_cmd',
            qos_profile=10
        )
        self.status_pub = self.new_publisher(
            Bool, f"/carla/{self.role_name}/status",
            qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

    def run(self):
        """
        Starts the main loop of the node
        :return:
        """
        self.status_pub.publish(True)
        self.loginfo('Acting node running')

        def loop(timer_event=None):
            """
            Main loop of the acting node
            :param timer_event: Timer event from ROS
            :return:
            """
            message = CarlaEgoVehicleControl()

            # set throttle to 0.1
            message.throttle = 0.3
            message.gear = 1
            message.brake = 0
            message.hand_brake = False
            message.manual_gear_shift = False

            message.header.stamp = roscomp.ros_timestamp(self.get_time(),
                                                         from_sec=True)
            self.control_publisher.publish(message)

        self.new_timer(self.control_loop_rate, loop)
        self.spin()


def main(args=None):
    """
      main function starts the acting node
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
