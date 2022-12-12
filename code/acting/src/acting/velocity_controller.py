#!/usr/bin/env python
import ros_compatibility as roscomp
from carla_msgs.msg import CarlaSpeedometer
from ros_compatibility.node import CompatibleNode
from rospy import Publisher, Subscriber
from simple_pid import PID
from std_msgs.msg import Float32


class VelocityController(CompatibleNode):
    """
    This node controls the velocity of the vehicle.
    For this it uses a PID controller
    """

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

        self.__current_velocity: float = None
        self.__max_velocity: float = None

    def run(self):
        """
        Starts the main loop of the node
        :return:
        """
        self.loginfo('VehicleController node running')
        pid = PID(0.25, 0, 0.1)  # values from paf21-2 todo: tune

        def loop(timer_event=None):
            """
            Calculates the result of the PID controller and publishes it.
            :param timer_event: Timer event from ROS
            :return:
            """
            if self.__max_velocity is None:
                self.logdebug("VehicleController hasn't received max_velocity"
                              " yet and can therefore not publish a "
                              "throttle value")
            if self.__current_velocity is None:
                self.logdebug("VehicleController hasn't received "
                              "current_velocity yet and can therefore not"
                              "publish a throttle value")
            if self.__max_velocity < 0:
                self.logerr("Velocity controller doesn't support backward "
                            "driving yet.")
                raise Exception("Velocity controller doesn't support backward "
                                "driving yet.")
            pid.setpoint = self.__max_velocity
            throttle = pid(self.__current_velocity)
            throttle = max(throttle, -1.0)  # ensures that throttle >= -1
            throttle = min(throttle, 1.0)  # ensures that throttle <= 1
            self.throttle_pub.publish(throttle)

        self.new_timer(self.control_loop_rate, loop)
        self.spin()

    def __get_current_velocity(self, data: CarlaSpeedometer):
        self.__current_velocity = float(data.speed)

    def __get_max_velocity(self, data: Float32):
        self.__max_velocity = float(data.data)


def main(args=None):
    """
    Main function starts the node
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
