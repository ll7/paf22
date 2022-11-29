#!/usr/bin/env python
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from carla_msgs.msg import CarlaEgoVehicleControl
from rospy import Publisher
from ros_compatibility.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, Header


class Acting(CompatibleNode):
    def __init__(self):
        super(Acting, self).__init__('acting')
        self.loginfo('Acting node started')

        self.control_loop_rate = self.get_param('control_loop_rate', 0.1)
        self.role_name = self.get_param('role_name', 'ego_vehicle')

        self.control_publisher: Publisher = self.new_publisher(
            CarlaEgoVehicleControl,
            f'/carla/{self.role_name}/vehicle_control_cmd',
            qos_profile=1
        )
        self.status_pub = self.new_publisher(
            Bool, f"/carla/{self.role_name}/status",
            qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        self.status_pub.publish(True)

    def run(self):
        self.loginfo('Acting node running')

        def loop(timer_event=None):
            self.loginfo('Acting node loop')
            message = CarlaEgoVehicleControl()

            # create header with correct timestamp
            header = Header()
            header.frame_id = 'map'
            header.stamp = roscomp.ros_timestamp(sec=self.get_time(),
                                                 from_sec=True)
            message.header = header

            # set throttle to 0.1
            message.throttle = 0.1
            self.control_publisher.publish(message)
            self.loginfo('Acting message published')
            self.loginfo(message)
            self.loginfo(self.get_time())

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
