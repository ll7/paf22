#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from lidar_utility import LidarUtility


def callback(data):
    lidar_utility = LidarUtility()
    lidar_utility.getDepthAtPoint(0, 0)


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('lidar_debugger', anonymous=True)

    rospy.Subscriber("/carla/hero/LIDAR", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
