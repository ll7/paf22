#!/usr/bin/env python
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaSpeedometer
from rospy import Publisher, Subscriber
from ros_compatibility.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, Float32

PURE_PURSUIT_CONTROLLER: int = 1
STANLEY_CONTROLLER: int = 2


# todo: docs
class VehicleController(CompatibleNode):
    """
    This node is responsible for collecting all data needed for the
    vehicle_control_cmd and sending it.
    The node also decides weather to use the stanley or pure pursuit
    controller.
    If the node receives an emergency msg, it will bring the vehicle to a stop
    and send an emergency msg with data = False back, after the velocity of the
    vehicle has reached 0.
    """
    def __init__(self):
        super(VehicleController, self).__init__('vehicle_controller')
        self.loginfo('VehicleController node started')

        self.control_loop_rate = self.get_param('control_loop_rate', 0.05)
        self.role_name = self.get_param('role_name', 'ego_vehicle')

        self.control_publisher: Publisher = self.new_publisher(
            CarlaEgoVehicleControl,
            f'/carla/{self.role_name}/vehicle_control_cmd',
            qos_profile=10
        )
        self.status_pub: Publisher = self.new_publisher(
            Bool,
            f"/carla/{self.role_name}/status",
            qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        self.emergency_pub: Publisher = self.new_publisher(
            Bool,
            f"/carla/{self.role_name}/emergency",
            qos_profile=10
        )

        self.emergency_sub: Subscriber = self.new_subscription(
            Bool,
            f"/carla/{self.role_name}/emergency",
            self.__emergency_break,
            qos_profile=10)

        self.velocity_sub: Subscriber = self.new_subscription(
            CarlaSpeedometer,
            f"/carla/{self.role_name}/Speed",
            self.__get_velocity,
            qos_profile=1)

        self.throttle_sub: Subscriber = self.new_subscription(
            Float32,
            f"/carla/{self.role_name}/throttle",
            self.__set_throttle,
            qos_profile=1)

        self.pure_pursuit_steer_sub: Subscriber = self.new_subscription(
            Float32,
            f"/carla/{self.role_name}/pure_pursuit_steer",
            self.__set_pure_pursuit_steer,
            qos_profile=1)

        self.stanley_steer_sub: Subscriber = self.new_subscription(
            Float32,
            f"/carla/{self.role_name}/stanley_steer",
            self.__set_stanley_steer,
            qos_profile=1)

        self.__emergency: bool = False
        self.__throttle: float = 0
        self.__pure_pursuit_steer: float = 0
        self.__stanley_steer: float = 0

    def run(self):
        """
        Starts the main loop of the node and send a status msg.
        :return:
        """
        self.status_pub.publish(True)
        self.loginfo('VehicleController node running')

        def loop(timer_event=None) -> None:
            """
            Collects all data received and sends a CarlaEgoVehicleControl msg.
            :param timer_event: Timer event from ROS
            :return:
            """
            # self.loginfo('Vehicle Controller Running')

            controller = self.__choose_controller()
            if controller == PURE_PURSUIT_CONTROLLER:
                self.logdebug('Using PURE_PURSUIT_CONTROLLER')
                steer = self.__pure_pursuit_steer
            elif controller == STANLEY_CONTROLLER:
                self.logdebug('Using STANLEY_CONTROLLER')
                steer = self.__stanley_steer
            else:
                self.logerr("Vehicle Controller couldn't find requested "
                            "controller.")
                raise Exception("Requested Controller not found")

            message = CarlaEgoVehicleControl()
            message.reverse = False
            if self.__throttle > 0:  # todo: driving backwards?
                message.brake = 0
                message.throttle = self.__throttle
            else:
                message.throttle = 0
                message.brake = abs(self.__throttle)

            message.hand_brake = False
            message.manual_gear_shift = False
            message.steer = steer
            # message.throttle = 5  # for testing todo: delete
            message.gear = 1
            message.header.stamp = roscomp.ros_timestamp(self.get_time(),
                                                         from_sec=True)
            self.control_publisher.publish(message)

        self.new_timer(self.control_loop_rate, loop)
        self.spin()

    def __emergency_break(self, data) -> None:
        """
        Executes emergency stop
        :param data:
        :return:
        """
        if not data.data:  # not an emergency
            return
        if self.__emergency:  # emergency was already triggered
            return
        self.loginfo("Emergency breaking engaged")
        self.__emergency = True
        message = CarlaEgoVehicleControl()
        message.throttle = 1
        message.steer = 1  # todo: maybe use 30 degree angle
        message.brake = 1
        message.reverse = True
        message.hand_brake = True
        message.manual_gear_shift = False
        message.header.stamp = roscomp.ros_timestamp(self.get_time(),
                                                     from_sec=True)
        self.control_publisher.publish(message)

    def __get_velocity(self, data: CarlaSpeedometer) -> None:
        """
        Ends emergency if vehicle velocity reaches 0. Sends emergency msg
        with data = False after stopping.
        :param data:
        :return:
        """
        if not self.__emergency:  # nothing to do in this case
            return
        if data.speed < 0.1:  # vehicle has come to a stop
            self.__emergency = False
            message = CarlaEgoVehicleControl()
            message.throttle = 0
            message.steer = 0
            message.brake = 1
            message.reverse = False
            message.hand_brake = False
            message.manual_gear_shift = False
            message.header.stamp = roscomp.ros_timestamp(self.get_time(),
                                                         from_sec=True)
            self.control_publisher.publish(message)

            self.loginfo("Emergency breaking disengaged "
                         "(Emergency breaking has been executed successfully)")
            self.emergency_pub.publish(
                Bool(False))  # todo: publish multiple times?

    def __set_throttle(self, data):
        self.__throttle = data.data

    def __set_pure_pursuit_steer(self, data):
        self.__pure_pursuit_steer = data.data

    def __set_stanley_steer(self, data):
        self.__stanley_steer = data.data

    def __choose_controller(self) -> int:  # todo: implement
        """
        Chooses with steering controller to use
        :return:
        """
        return STANLEY_CONTROLLER


def main(args=None):
    """
    Main function starts the node
    :param args:
    """
    roscomp.init('vehicle_controller', args=args)

    try:
        node = VehicleController()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == '__main__':
    main()
