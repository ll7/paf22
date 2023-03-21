#!/usr/bin/env python
import rospy
import ros_numpy
from sensor_msgs.msg import Image


class LidarDepthBuffer():

    last_img = None

    def callback(self, data):
        img = ros_numpy.numpify(data)

        rospy.loginfo(self.last_img)

        if self.last_img is not None:
            mask = (img > 5000)
            self.last_img[mask] = img[mask]

        else:
            self.last_img = img

        self.pub_depth.publish(
            ros_numpy.msgify(Image, self.last_img, encoding='16UC1')
        )

    def listener(self):
        rospy.init_node('lidar_depth_buffer')

        self.pub_depth = rospy.Publisher('/depth_buffered_out', Image)

        rospy.Subscriber("/depth_out", Image, self.callback)
        rospy.spin()


if __name__ == '__main__':
    lidar_depth_buffer = LidarDepthBuffer()
    lidar_depth_buffer.listener()
