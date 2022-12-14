#!/usr/bin/env python
import rospy
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from carla_msgs.msg import CarlaRoute
from std_msgs.msg import Bool
from ros_compatibility.qos import QoSProfile, DurabilityPolicy


class NavManager(CompatibleNode):

    def __init__(self):
        super(NavManager, self).__init__('Navigation_Data')
        self.loginfo('Navigation_Data node started')
        self.new_subscription(msg_type=CarlaRoute,
                              topic='/carla/hero/global_plan',
                              callback=nav_update, qos_profile=10)

        rospy.Subscriber(name='/carla/hero/global_plan', data_class=CarlaRoute,
                         callback=nav_update, queue_size=10)

        self.status_pub = self.new_publisher(
            Bool, "/carla/hero/status",
            qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

    def run(self):
        """
        Starts the main loop of the node
        :return:
        """
        self.loginfo('Navigation node running')
        self.status_pub.publish(True)

        def loop(timer_event=None):
            """
            Main loop of the acting node
            :param timer_event: Timer event from ROS
            :return:
            """
            pass
        self.new_timer(0.05, loop)
        self.spin()


def nav_update(data):
    """_summary_

    :param data:
    :return: _description_
    """
    roscomp.loginfo(f"nav data: {data}")
    print(f"nav data: {data}")


if __name__ == "__main__":
    """
          main function starts the MapManager node
          :param args:
        """
    roscomp.init('MapManager')

    try:
        node = NavManager()
        # node.run()

        while True:
            pass
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()
