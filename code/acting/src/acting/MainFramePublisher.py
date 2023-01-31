#!/usr/bin/env python
from math import cos, sin, radians

import ros_compatibility as roscomp
import rospy
from geometry_msgs.msg import PoseStamped
from ros_compatibility.node import CompatibleNode
import tf
from scipy.spatial.transform import Rotation as R


class MainFramePublisher(CompatibleNode):

    def __init__(self):
        """
        This node handles the translation from the static main frame to the
        moving hero frame. The hero frame always moves and rotates as the
        ego vehicle does. The hero frame is used by sensors like the lidar.
        Rviz also uses the hero frame. The main frame is used for planning.
        """
        super(MainFramePublisher, self).__init__('main_frame_publisher')
        self.loginfo('MainFramePublisher node started')

        self.control_loop_rate = self.get_param('control_loop_rate', 0.05)
        self.role_name = self.get_param('role_name', 'ego_vehicle')
        self.current_pos: PoseStamped = PoseStamped()

        self.current_pos_subscriber = self.new_subscription(
            PoseStamped,
            "/carla/" + self.role_name + "/current_pos",
            self.get_current_pos,
            qos_profile=1)

    def run(self):
        self.loginfo('MainFramePublisher node running')
        br = tf.TransformBroadcaster()

        def loop(timer_event=None):
            if self.current_pos is None:
                # conversion only works if pos is known
                return
            # Invert Translation and Rotation to transform from main
            # to hero frame
            curr_rot = self.current_pos.pose.orientation
            rot_quat = [curr_rot.x, curr_rot.y, curr_rot.z, curr_rot.w]
            if all(v == 0 for v in rot_quat):
                self.loginfo("Invalid rotation data")
                rot_quat = [1, 0, 0, 0]
            rot_deg = -R.from_quat(rot_quat).as_euler("xyz", degrees=True)[
                0] + 90
            # self.loginfo(R.from_quat(rot_quat).as_euler("xyz", degrees=True))
            pos = [0, 0, 0]
            pos[0] = cos(radians(rot_deg)) * \
                self.current_pos.pose.position.x - \
                sin(radians(rot_deg)) * self.current_pos.pose.position.y
            pos[1] = sin(radians(rot_deg)) * \
                self.current_pos.pose.position.x + \
                cos(radians(rot_deg)) * self.current_pos.pose.position.y
            pos[2] = self.current_pos.pose.position.z
            pos[0] = pos[0] * -1
            pos[1] = pos[1] * -1
            rot_quat = R.from_euler("xyz", [0, 0, rot_deg], degrees=True) \
                .as_quat()
            br.sendTransform(pos,
                             rot_quat,
                             rospy.Time.now(),
                             "global",
                             "hero",
                             )

        self.new_timer(self.control_loop_rate, loop)
        self.spin()

    def get_current_pos(self, data: PoseStamped):
        self.current_pos = data


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
