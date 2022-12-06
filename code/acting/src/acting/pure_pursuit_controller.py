#!/usr/bin/env python
import ros_compatibility as roscomp
# from carla_msgs.msg import CarlaGnssRoute
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile, DurabilityPolicy
from rospy import Publisher, Subscriber
from std_msgs.msg import Float32, Bool


# todo: docs
class PurePursuitController(CompatibleNode):
    def __init__(self):
        super(PurePursuitController, self).__init__('pure_pursuit_controller')
        self.loginfo('PurePursuitController node started')

        self.control_loop_rate = self.get_param('control_loop_rate', 0.05)
        self.role_name = self.get_param('role_name', 'ego_vehicle')

        self.stanley_steer_sub: Subscriber = self.new_subscription(
            Float32,
            f"/carla/{self.role_name}/max_velocity",
            self.__get_max_velocity,
            qos_profile=1)

        # self.position_sub: Subscriber = self.new_subscription(
        #     GNSSMeasurement,
        #     f"/carla/{self.role_name}/Speed",
        #     self.__get_velocity,
        #     qos_profile=1)

        self.pure_pursuit_steer_pub: Publisher = self.new_publisher(
            Float32,
            f"/carla/{self.role_name}/pure_pursuit_steer",
            qos_profile=1)

        self.status_pub = self.new_publisher(  # todo: delete (this is only
            # necessary if vehicle controller isn't running)
            Bool, f"/carla/{self.role_name}/status",
            qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        self.__current_velocity: float = 0
        self.__max_velocity: float = 0

    def run(self):
        """
        Starts the main loop of the node
        :return:
        """
        self.loginfo('PurePursuitController node running')
        self.status_pub.publish(True)  # todo: delete (this is only
        # necessary if vehicle controller isn't running)

        def loop(timer_event=None):
            """
            Main loop of the acting node
            :param timer_event: Timer event from ROS
            :return:
            """
            pass

        self.new_timer(self.control_loop_rate, loop)
        self.spin()


def main(args=None):
    """
      main function starts the acting node
      :param args:
    """
    roscomp.init('pure_pursuit_controller', args=args)

    try:
        node = PurePursuitController()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == '__main__':
    main()
