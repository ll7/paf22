#!/usr/bin/env python
import ros_compatibility as roscomp
import rospy
from geometry_msgs.msg import Pose
from ros_compatibility.node import CompatibleNode
import tf

from coordinate_transformation import \
    CoordinateTransformer, GeoRef
from sensor_msgs.msg import NavSatFix


class MainFramePublisher(CompatibleNode):

    def __init__(self):
        super(MainFramePublisher, self).__init__('main_frame_publisher')
        self.loginfo('MainFramePublisher node started')

        self.control_loop_rate = self.get_param('control_loop_rate', 0.05)
        self.role_name = self.get_param('role_name', 'ego_vehicle')

        self.pos_counter = 0
        self.pos_average = [0, 0, 0]
        self.transformer = CoordinateTransformer()
        gps_ref = GeoRef.TOWN12
        lat0 = gps_ref.value[0]
        lon0 = gps_ref.value[1]
        h0 = gps_ref.value[2]
        self.transformer.set_gnss_ref(lat0, lon0, h0)
        self.current_pos: Pose = Pose()

        self.gnss_subscriber = self.new_subscription(
            NavSatFix,
            "/carla/" + self.role_name + "/GPS",
            self.output_average_gps_2_xyz,
            qos_profile=1)

    def run(self):
        self.loginfo('MainFramePublisher node running')
        br = tf.TransformBroadcaster()

        def loop(timer_event=None):
            self.loginfo((self.current_pos.position.x,
                          self.current_pos.position.y,
                          self.current_pos.position.z))

            br.sendTransform((-self.current_pos.position.x,
                              -self.current_pos.position.y,
                              -self.current_pos.position.z),
                             (0.0, 0.0, 0.0, 1.0),  # todo: add rot
                             rospy.Time.now(),
                             "main",
                             "hero",
                             )

        self.new_timer(self.control_loop_rate, loop)
        self.spin()

    def update_pose(self, x, y, z):
        """
        Update the current position and publishes the new position.
        :param x:
        :param y:
        :param z:
        :return:
        """

        temp_pose = Pose()
        temp_pose.position.x = x
        temp_pose.position.y = y
        temp_pose.position.z = z

        temp_pose.orientation.x = 0.0
        temp_pose.orientation.y = 0.0
        temp_pose.orientation.z = 0.0
        temp_pose.orientation.w = 0.0

        self.current_pos = temp_pose

    def output_gps_2_xyz(self, data: NavSatFix):
        """
        Transforms GPS coordinates into local coordinates
        using the CoordinateTransformer transformer.
        The current position is then updated and published
        in the PoseStamped format.
        :param data: message according to NavSatFix definition
        :return:
        """
        if self.pos_counter % 4 == 0:
            lat = data.latitude
            lon = data.longitude
            h = data.altitude
            x, y, z = self.transformer.gnss_to_xyz(self, lat, lon, h)
            self.update_pose(x, y, z)
            # self.loginfo(f"x: {x}\t y: {y}\t z:{z}")

        self.pos_counter += 1

    def output_average_gps_2_xyz(self, data: NavSatFix):
        """
        Transforms GPS coordinates into local coordinates
        using the CoordinateTransformer transformer.
        The current position is then updated and published
        in the PoseStamped format.
        :param data: message according to NavSatFix definition
        :return:
        """

        lat = data.latitude
        lon = data.longitude
        h = data.altitude
        x, y, z = self.transformer.gnss_to_xyz(lat, lon, h)

        self.pos_average[0] += x
        self.pos_average[1] += y
        self.pos_average[2] += z

        if self.pos_counter % 5 == 0:  # todo: do this every step
            x1 = self.pos_average[0] / 5
            y1 = self.pos_average[1] / 5
            z1 = self.pos_average[2] / 5
            self.pos_average = [0, 0, 0]
            self.update_pose(x1, y1, z1)
            # self.loginfo(f"x: {x1}\t y: {y1}\t z:{z1}")

        self.pos_counter += 1


def main(args=None):
    """
    Main function starts the node
    :param args:
    """
    roscomp.init('main_frame_publisher', args=args)

    try:
        node = MainFramePublisher()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == '__main__':
    main()
