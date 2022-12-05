#!/usr/bin/env python
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from carla_msgs.msg import CarlaEgoVehicleControl
from rospy import Publisher
from ros_compatibility.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, Float32

PURE_PURSUIT_CONTROLLER = 1
STANLEY_CONTROLLER = 2


class VehicleController(CompatibleNode):
    def __init__(self):
        super(VehicleController, self).__init__('acting')
        self.loginfo('VehicleController node started')

        self.control_loop_rate = self.get_param('control_loop_rate', 0.05)
        self.role_name = self.get_param('role_name', 'ego_vehicle')

        self.control_publisher: Publisher = self.new_publisher(
            CarlaEgoVehicleControl,
            f'/carla/{self.role_name}/vehicle_control_cmd',
            qos_profile=10
        )
        self.status_pub: Publisher = self.new_publisher(
            Bool, f"/carla/{self.role_name}/status",
            qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        self.status_pub: Publisher = self.new_publisher(
            Bool, f"/carla/{self.role_name}/status",
            qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        self.emergency_pub: Publisher = self.new_publisher(
            Bool, f"/carla/{self.role_name}/emergency",
            qos_profile=10
        )

        self.Subsciber(f"/carla/{self.role_name}/emergency", Bool,
                       self.emergency_break)

        self.Subsciber(f"/carla/{self.role_name}/CarlaEgoVehicleStatus",
                       Float32, self.get_velocity)

        self.Subsciber(f"/carla/{self.role_name}/pure_pursuit_throttle",
                       Float32, self.set_pure_pursuit_throttle)

        self.Subsciber(f"/carla/{self.role_name}/pure_pursuit_steer", Float32,
                       self.set_pure_pursuit_steer)

        self.Subsciber(f"/carla/{self.role_name}/stanley_throttle", Float32,
                       self.set_stanley_throttle)

        self.Subsciber(f"/carla/{self.role_name}/stanley_steer", Float32,
                       self.set_stanley_steer)

        self.emergency: bool = False
        self.pure_pursuit_throttle: float = 0
        self.pure_pursuit_steer: float = 0
        self.stanley_throttle: float = 0
        self.stanley_steer: float = 0

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
            if self.choose_controller() == PURE_PURSUIT_CONTROLLER:
                throttle = self.pure_pursuit_throttle
                steer = self.pure_pursuit_steer
            elif self.choose_controller() == STANLEY_CONTROLLER:
                throttle = self.stanley_throttle
                steer = self.stanley_steer
            else:
                raise Exception("Controller not found")

            message = CarlaEgoVehicleControl()
            message.reverse = False
            if self.throttle > 0:  # todo: driving backwards?
                message.brake = 0
                message.throttle = throttle
            else:
                message.throttle = 0
                message.brake = abs(throttle)

            message.hand_brake = False
            message.manual_gear_shift = False
            message.steer = steer
            # message.throttle = 5  # for testing todo: delete
            message.header.stamp = roscomp.ros_timestamp(self.get_time(),
                                                         from_sec=True)
            self.control_publisher.publish(message)

        self.new_timer(self.control_loop_rate, loop)
        self.spin()

    def emergency_break(self, data):
        if not data.data:  # not an emergency
            return
        self.emergency = True
        message = CarlaEgoVehicleControl()
        message.throttle = 1
        message.steer = 1  # todo: look up 30 degree angle
        message.brake = 1
        message.reverse = True
        message.hand_brake = True
        message.manual_gear_shift = False
        message.header.stamp = roscomp.ros_timestamp(self.get_time(),
                                                     from_sec=True)
        self.control_publisher.publish(message)

    def get_velocity(self, data):
        if not self.emergency:  # nothing to do in this case
            return
        if data.data < 0.1:  # vehicle has come to a stop
            self.emergency = False
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

            self.emergency_pub.publish(
                Bool(False))  # todo: publish multiple times?

    def set_pure_pursuit_throttle(self, data):
        self.pure_pursuit_throttle = data.data

    def set_pure_pursuit_steer(self, data):
        self.pure_pursuit_steer = data.data

    def set_stanley_throttle(self, data):
        self.stanley_throttle = data.data

    def set_stanley_steer(self, data):
        self.stanley_steer = data.data

    def choose_controller(self):  # todo: implement
        return PURE_PURSUIT_CONTROLLER


def main(args=None):
    """
      main function starts the acting node
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
