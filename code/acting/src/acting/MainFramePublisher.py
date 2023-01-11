#!/usr/bin/env python
import ros_compatibility as roscomp
import rospy
from geometry_msgs.msg import Pose
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
        self.current_pos: Pose = None

        self.current_pos_subscriber = self.new_subscription(
            Pose,
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
            pos = (-self.current_pos.position.x,
                   -self.current_pos.position.y,
                   -self.current_pos.position.z)
            curr_rot = self.current_pos.orientation
            rot_quat = [curr_rot.x, curr_rot.y, curr_rot.z, curr_rot.w]
            invert_rot_quat = R.from_quat(rot_quat).inv().as_quat()
            br.sendTransform(pos,
                             invert_rot_quat,
                             rospy.Time.now(),
                             "main",
                             "hero",
                             )

        self.new_timer(self.control_loop_rate, loop)
        self.spin()

    def get_current_pos(self, data: Pose):
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
