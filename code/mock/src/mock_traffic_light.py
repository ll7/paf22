#!/usr/bin/env python
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from rospy import Publisher
# from std_msgs.msg import Float32
from mock.msg import Traffic_light


class MockTrafficLightPublisher(CompatibleNode):
    """
    This node publishes traffic light information. It can be used for testing.
    TODO functionality
    """
    def __init__(self):
        super(MockTrafficLightPublisher, self).\
            __init__('trafficLightMock')

        self.control_loop_rate = self.get_param('control_loop_rate', 10)
        self.role_name = self.get_param('role_name', 'ego_vehicle')
        # self.enabled = self.get_param('enabled', False)

        self.traffic_light_pub: Publisher = self.new_publisher(
            Traffic_light,
            f"/carla/{self.role_name}/traffic_light",
            qos_profile=1)
        self.velocity = 7
        self.delta_velocity = 0.25
        self.max_velocity = 10
        self.min_velocity = 5
        self.__dv = self.delta_velocity

    def run(self):
        """
        Starts the main loop of the node
        :return:
        """
        # if not self.enabled:
        #     return
        self.loginfo('TrafficLightmock node running')

        def loop(timer_event=None):
            """
            Publishes dummy velocity
            :param timer_event: Timer event from ROS
            :return:
            """
            # self.loginfo('Published dummy velocity: ' + str(self.velocity))
            msg = Traffic_light()
            msg.color = "green"
            msg.distance = 100.0
            self.velocity_pub.publish(msg)
            # if self.velocity > self.max_velocity:
            #     self.__dv = -self.delta_velocity
            # if self.velocity < self.min_velocity:
            #     self.__dv = self.delta_velocity
            # self.velocity += self.__dv
        self.new_timer(self.control_loop_rate, loop)
        self.spin()


def main(args=None):
    """
    Main function starts the node
    :param args:
    """
    roscomp.init('velocity_publisher_dummy', args=args)

    try:
        node = MockTrafficLightPublisher()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == '__main__':
    main()
