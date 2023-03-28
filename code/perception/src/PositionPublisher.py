#!/usr/bin/env python

"""
This node tests the subscription side of the dummy trajectory message.
It therefore receives a nav_msgs/Path msg.
"""

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from coordinate_transformation import CoordinateTransformer, GeoRef, \
    quat_to_heading
import numpy as np
import rospy


class PositionPublisher(CompatibleNode):
    """
    Creates a node that publishes the current Pose of the vehicle.
    """

    def __init__(self):
        """
        Constructor
        :return:
        """

        super(PositionPublisher, self).__init__('position_publisher')
        self.loginfo("PositionPublisher node started")

        self.transformer = CoordinateTransformer(GeoRef.TOWN12)

        # current_pos ist the final PoseStamped that will be published
        self.current_pos_gps: PoseWithCovarianceStamped = \
            PoseWithCovarianceStamped()

        # basic info
        self.role_name = self.get_param("role_name", "hero")
        self.control_loop_rate = self.get_param("control_loop_rate", "0.05")
        self.publish_loop_rate = 0.05  # 20Hz rate like the sensors

        # Subscriber
        self.pos_filtered_subscriber = self.new_subscription(
            PoseWithCovarianceStamped,
            "/robot_pose_ekf/odom_combined",
            self.update_pos_filtered_data,
            qos_profile=1)

        # Publisher
        self.pos_publisher = self.new_publisher(
            PoseStamped,
            "/paf/" + self.role_name + "/current_pos",
            qos_profile=1)

        self.avg_xyz = np.zeros((5, 3))
        self.__publish_counter = 0

    def update_pos_filtered_data(self, data: PoseWithCovarianceStamped):
        self.current_pos_gps = data

    def update_current_pos(self):
        """
        Updates the current position based on the most upto date
        IMU, Speedometer and GNSS sensor data.
        :return:
        """
        # self.loginfo("updating pos data")
        x = self.current_pos_gps.pose.pose.position.x
        y = self.current_pos_gps.pose.pose.position.y
        z = self.current_pos_gps.pose.pose.position.z
        # x, y, z = self.transformer.gnss_to_xyz(lat, lon, alt)

        orientation_quat = self.current_pos_gps.pose.pose.orientation
        self.current_heading = quat_to_heading(orientation_quat)
        # self.loginfo(degrees(self.current_heading))
        # self.loginfo(self.current_heading)

        temp_pose: PoseStamped = PoseStamped()

        temp_pose.header.stamp = rospy.Time.now()
        temp_pose.header.frame_id = "global"
        temp_pose.pose.position.x = x
        temp_pose.pose.position.y = y
        temp_pose.pose.position.z = z
        temp_pose.pose.orientation = orientation_quat

        return temp_pose

    def publish_current_pos(self):
        """
        Calls for an update of the current position,
        and then publishes the current position.
        :return:
        """
        # self.loginfo("publishing data")
        temp_pos = self.update_current_pos()
        pos = np.matrix([temp_pos.pose.position.x,
                        temp_pos.pose.position.y,
                        temp_pos.pose.position.z])
        self.avg_xyz = np.roll(self.avg_xyz, -1, axis=0)
        self.avg_xyz[-1] = pos

        # compute average of each column
        avg_x, avg_y, avg_z = np.mean(self.avg_xyz, axis=0)
        temp_pos.pose.position.x = avg_x
        temp_pos.pose.position.y = avg_y
        temp_pos.pose.position.z = avg_z
        self.pos_publisher.publish(temp_pos)

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
        node = PositionPublisher()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
