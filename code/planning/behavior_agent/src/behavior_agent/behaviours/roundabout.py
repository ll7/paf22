import py_trees
import numpy as np
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from custom_carla_msgs.srv import UpdateLocalPath, TrafficOnLanelet

import rospy
import math


class Approach(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Approach, self).__init__(name)

    def setup(self, timeout):
        self.target_speed_pub = rospy.Publisher("/psaf/ego_vehicle/target_speed",
                                                Float64, queue_size=1)
        rospy.wait_for_service('update_local_path')
        self.update_local_path = rospy.ServiceProxy("update_local_path", UpdateLocalPath)
        self.Successs = False
        return True

    def initialise(self):
        self.update_local_path(approach_roundabout=True)
        self.blackboard = py_trees.blackboard.Blackboard()
        rospy.loginfo("Starting to approach Roundabout")

    def update(self):

        msg = self.blackboard.get("/psaf/ego_vehicle/distance_next_roundabout")
        self.odo = self.blackboard.get("/carla/ego_vehicle/odometry")
        if msg is not None and msg.isRoundabout:
            dist_x = msg.entry_point.x - self.odo.pose.pose.position.x
            dist_y = msg.entry_point.y - self.odo.pose.pose.position.y
            dist = math.sqrt(dist_x ** 2 + dist_y ** 2)
            if dist < 30:
                v = max(5., (dist / 30)**1.5 * 50)
                self.target_speed_pub.publish(v)

        self.speed = np.sqrt(
            self.odo.twist.twist.linear.x ** 2 + self.odo.twist.twist.linear.y ** 2 +
            self.odo.twist.twist.linear.z ** 2) * 3.6
        if dist < 5:
            self.target_speed_pub.publish(0.0)
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


class Wait(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Wait, self).__init__(name)

    def setup(self, timeout):
        self.target_speed_pub = rospy.Publisher("/psaf/ego_vehicle/target_speed",
                                                Float64, queue_size=1)
        rospy.wait_for_service('check_lanelet_free')
        self.lanelet_free = rospy.ServiceProxy("check_lanelet_free", TrafficOnLanelet)

        self.Successs = True
        return True

    def initialise(self):
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        first_lanelet_roundabout = self.blackboard.get("/psaf/ego_vehicle/first_lanelet_roundabout")
        if first_lanelet_roundabout is not None:
            success_lanelet_free = self.lanelet_free(isRoundabout=True,
                                                     lanelet_id=first_lanelet_roundabout.data)
            rospy.loginfo(f"success_lanelet_free = {success_lanelet_free.Free}")
            if success_lanelet_free.Free:
                return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


class Enter(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Enter, self).__init__(name)

    def setup(self, timeout):
        self.target_speed_pub = rospy.Publisher("/psaf/ego_vehicle/target_speed",
                                                Float64, queue_size=1)
        self.Successs = False
        return True

    def initialise(self):
        rospy.loginfo("Entering Enter")
        self.target_speed_pub.publish(30.0)
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        msg = self.blackboard.get("/psaf/ego_vehicle/distance_exit_roundabout")
        self.odo = self.blackboard.get("/carla/ego_vehicle/odometry")
        dist = np.inf
        if msg is not None:
            dist_x = msg.x - self.odo.pose.pose.position.x
            dist_y = msg.y - self.odo.pose.pose.position.y
            dist = math.sqrt(dist_x ** 2 + dist_y ** 2)
        if dist < 8:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


class Leave(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Leave, self).__init__(name)

    def setup(self, timeout):
        self.target_speed_pub = rospy.Publisher("/psaf/ego_vehicle/target_speed",
                                                Float64, queue_size=1)
        self.Successs = False
        rospy.wait_for_service('update_local_path')
        self.update_local_path = rospy.ServiceProxy("update_local_path", UpdateLocalPath)
        return True

    def initialise(self):
        self.blackboard = py_trees.blackboard.Blackboard()
        self.update_local_path(leave_intersection=True)
        self.target_speed_pub.publish(50.0)

    def update(self):
        if self.Successs:
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))
