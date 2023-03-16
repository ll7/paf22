#!/usr/bin/env python
import rospy
import ros_numpy
import numpy as np
import lidar_filter_utility
from sensor_msgs.msg import PointCloud2, Range


class LidarDistance():

    def callback(self, data):
        """ Callback function to process PointCloud2 and
            publish a Range message from the contained points
            and a PointCloud2

        :param data:
        :return:
        """
        coordinates = ros_numpy.point_cloud2.pointcloud2_to_array(data)

        # https://stackoverflow.com/questions/44295375/how-to-slice-a-numpy-ndarray-made-up-of-numpy-void-numbers
        bit_mask = lidar_filter_utility.bounding_box(
            coordinates,
            max_x=rospy.get_param('~max_x', np.inf),
            min_x=rospy.get_param('~min_x', np.inf),
            max_y=rospy.get_param('~max_y', np.inf),
            min_y=rospy.get_param('~min_y', np.inf),
            max_z=rospy.get_param('~max_z', np.inf),
            min_z=rospy.get_param('~min_z', np.inf),
        )

        rospy.loginfo(len(coordinates))
        # Filter coordinates based in generated bit_mask
        coordinates = coordinates[bit_mask]
        rospy.loginfo(len(coordinates))

        # Create pointcloud from manipulated data
        coordinates_manipulated = ros_numpy \
            .point_cloud2.array_to_pointcloud2(coordinates)
        coordinates_manipulated.header = data.header

        # Publish manipulated pointCloud2
        self.pub_pointcloud.publish(coordinates_manipulated)

        # https://stackoverflow.com/questions/1401712/how-can-the-euclidean-distance-be-calculated-with-numpy
        coordinates_xyz = np.array(
            lidar_filter_utility.remove_field_name(coordinates, 'intensity')
            .tolist()
        )
        distances = np.array(
            [np.linalg.norm(c - [0, 0, 0]) for c in coordinates_xyz])

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
        rospy.init_node('lidar_distance')

        self.pub_pointcloud = rospy.Publisher(
            rospy.get_param(
                '~point_cloud_topic',
                '/carla/hero/' + rospy.get_namespace() + '_filtered'
            ),
            PointCloud2
        )
        self.pub_range = rospy.Publisher(
            rospy.get_param(
                '~range_topic',
                '/carla/hero/' + rospy.get_namespace() + '_range'
            ),
            Range
        )

        rospy.Subscriber("/carla/hero/LIDAR", PointCloud2, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    lidar_distance = LidarDistance()
    lidar_distance.listener()
