#!/usr/bin/env python
import ros_compatibility as roscomp
from carla_msgs.msg import CarlaSpeedometer
from ros_compatibility.node import CompatibleNode
from rospy import Publisher, Subscriber
from simple_pid import PID
from std_msgs.msg import Float32


# todo: docs
class VelocityController(CompatibleNode):
    def __init__(self):
        super(VelocityController, self).__init__('velocity_controller')
        self.loginfo('VelocityController node started')

        self.control_loop_rate = self.get_param('control_loop_rate', 0.05)
        self.role_name = self.get_param('role_name', 'ego_vehicle')

        self.stanley_steer_sub: Subscriber = self.new_subscription(
            Float32,
            f"/carla/{self.role_name}/max_velocity",
            self.__get_max_velocity,
            qos_profile=1)

        self.velocity_sub: Subscriber = self.new_subscription(
            CarlaSpeedometer,
            f"/carla/{self.role_name}/Speed",
            self.__get_current_velocity,
            qos_profile=1)

        self.throttle_pub: Publisher = self.new_publisher(
            Float32,
            f"/carla/{self.role_name}/throttle",
            qos_profile=1)

        self.__current_velocity: float = 0
        self.__max_velocity: float = 0

    def run(self):
        """
        Starts the main loop of the node
        :return:
        """
        self.loginfo('VehicleController node running')
        pid = PID(0.25, 0, 0.1)  # values from paf21-2 todo: test

        def loop(timer_event=None):
            """
            Main loop of the acting node
            :param timer_event: Timer event from ROS
            :return:
            """
            if self.__max_velocity < 0:
                self.logerr("Velocity controller doesn't support backward "
                            "driving yet.")
                raise Exception("Velocity controller doesn't support backward "
                                "driving yet.")
            pid.setpoint = self.__max_velocity
            throttle = pid(self.__current_velocity)
            self.throttle_pub.publish(throttle)

        self.new_timer(self.control_loop_rate, loop)
        self.spin()

    def __get_current_velocity(self, data):
        self.__current_velocity = data

    def __get_max_velocity(self, data):
        self.__max_velocity = data


def main(args=None):
    """
      main function starts the acting node
      :param args:
    """
    roscomp.init('velocity_controller', args=args)

    try:
        node = VelocityController()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == '__main__':
    main()
