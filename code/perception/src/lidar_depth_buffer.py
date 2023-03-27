#!/usr/bin/env python
import rospy
import ros_numpy
from sensor_msgs.msg import Image


class LidarDepthBuffer():
    """ Node to override all Pixels which are greater than a lower limit.
        It takes a PointCloud2 message as input and outputs a PointCloud2
        message on which the described method was applied
    """

    # Store combined depth map
    last_img = None

    def callback(self, data):
        # make numpy array from depthmap
        img = ros_numpy.numpify(data)

        # create bitmask (True, if distance greater than 5000)
        # and override all pixels where the bitmask is True
        if self.last_img is not None:
            mask = (img > 5000)
            self.last_img[mask] = img[mask]

        # if we have no hostorical data yet just use the
        # values we got
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
