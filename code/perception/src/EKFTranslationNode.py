#!/usr/bin/env python

"""
This node tests the subscription side of the dummy trajectory message.
It therefore receives a nav_msgs/Path msg.
"""

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
# from carla_msgs.msg import CarlaSpeedometer
from sensor_msgs.msg import NavSatFix
# from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# import rospy


class EKFTranslation(CompatibleNode):
    """
    Translates to the top required by robot_pose_ekf.
    """

    def __init__(self):
        """
        Constructor
        :return:
        """

        super(EKFTranslation, self).__init__('ekf_translation')
        self.loginfo("EKF_Translation node started")

        # basic info
        self.role_name = self.get_param("role_name", "hero")
        self.control_loop_rate = self.get_param("control_loop_rate", "0.05")

        # Subscriber
        self.gnss_subscriber = self.new_subscription(
            NavSatFix,
            "/carla/" + self.role_name + "/GPS",
            self.update_gps_data,
            qos_profile=1)

        # Publisher
        # 2D Odometry (Maybe Speedometer?)
        self.ekf_odom_publisher = self.new_publisher(
            Odometry,
            "/carla/" + self.role_name + "/ekf_odom",
            qos_profile=1)

        # 3D Odomoetry (GPS)
        self.ekf_vo_publisher = self.new_publisher(
            Odometry,
            "/carla/" + self.role_name + "/ekf_vo",
            qos_profile=1)

    def run(self):
        """
        Control loop

        :return:
        """

        def loop(timer_event=None):
            self.publish_current_pos()

        self.new_timer(self.publish_loop_rate, loop)
        self.spin()


def main(args=None):
    """
    main function

    :param args:
    :return:
    """

    roscomp.init("position_publisher", args=args)
    try:
        node = EKFTranslation()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
