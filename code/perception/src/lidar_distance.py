#!/usr/bin/env python
import rospy
import ros_numpy
import numpy as np
import lidar_filter_utility
from sensor_msgs.msg import PointCloud2, Range

class LidarDistance():

    pub_pointcloud = rospy.Publisher('/carla/hero/LIDAR_filtered', PointCloud2)
    pub_range = rospy.Publisher('/carla/hero/LIDAR_range', Range)

    def callback(self, data):
        coordinates = ros_numpy.point_cloud2.pointcloud2_to_array(data)

        # https://stackoverflow.com/questions/44295375/how-to-slice-a-numpy-ndarray-made-up-of-numpy-void-numbers
        bit_mask = lidar_filter_utility.bounding_box(coordinates, max_x=1, min_x=-1, min_y=1, min_z=-1.5, max_z=0)

        rospy.loginfo(len(coordinates))
        # Filter coordinates based in generated bit_mask
        coordinates = coordinates[bit_mask]
        rospy.loginfo(len(coordinates))

        # Create pointcloud from manipulated data
        coordinates_manipulated = ros_numpy.point_cloud2.array_to_pointcloud2(coordinates)
        coordinates_manipulated.header = data.header

        # Publish manipulated pointCloud2
        self.pub_pointcloud.publish(coordinates_manipulated)

        # https://stackoverflow.com/questions/1401712/how-can-the-euclidean-distance-be-calculated-with-numpy
        coordinates_xyz = np.array(lidar_filter_utility.remove_field_name(coordinates, 'intensity').tolist())
        distances = np.array([np.linalg.norm(c - [0, 0, 0]) for c in coordinates_xyz])

        if len(distances) > 0:
            range_msg = Range()
            range_msg.max_range = max(distances)
            range_msg.min_range = min(distances)
            range_msg.range = min(distances)

            self.pub_range.publish(range_msg)

    def listener(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('lidar_distance', anonymous=True)

        rospy.Subscriber("/carla/hero/LIDAR", PointCloud2, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    lidar_distance = LidarDistance()
    lidar_distance.listener()
