#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from lidar_utility import LidarUtility

class LidarDebugger():
    lidar_utility = LidarUtility()

    def callback(self, data):
        #rospy.loginfo(data)
        self.lidar_utility.processLidarImage(data)

    def listener(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('lidar_debugger', anonymous=True)

        rospy.Subscriber("/carla/hero/Center/image", Image, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    lidar_debugger = LidarDebugger()
    lidar_debugger.listener()
