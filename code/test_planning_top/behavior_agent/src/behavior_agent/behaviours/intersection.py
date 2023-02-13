import py_trees
import numpy as np
from std_msgs.msg import Float64
# from nav_msgs.msg import Odometry
# from custom_carla_msgs.srv import UpdateLocalPath

import rospy

"""
Source: https://github.com/ll7/psaf2
"""


class Approach(py_trees.behaviour.Behaviour):
    """
    This behaviour is executed when the ego vehicle is in close proximity of
    an intersection and behaviours.road_features.intersection_ahead is
    triggered. It than handles the approaching the intersection, slowing the
    vehicle down appropriately.
    """
    def __init__(self, name):
        """
        Minimal one-time initialisation. A good rule of thumb is to only
        include the initialisation relevant for being able to insert this
        behaviour in a tree for offline rendering to dot graphs.
        Other one-time initialisation requirements should be met via the
        setup() method.
         :param name: name of the behaviour
        """
        super(Approach, self).__init__(name)

    def setup(self, timeout):
        """
        Delayed one-time initialisation that would otherwise interfere with
        offline rendering of this behaviour in a tree to dot graph or
        validation of the behaviour's configuration.

        This initializes the blackboard to be able to access data written to it
        by the ROS topics and the target speed publisher.
        :param timeout: an initial timeout to see if the tree generation is
        successful
        :return: True, as the set up is successful.
        """
        self.target_speed_pub = rospy.Publisher("/carla/hero/max_velocity",
                                                Float64, queue_size=1)
        # rospy.wait_for_service('update_local_path') # TODO is this necessary?
        # self.update_local_path =
        # rospy.ServiceProxy("update_local_path", UpdateLocalPath)
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def initialise(self):
        """
        When is this called?
        The first time your behaviour is ticked and anytime the status is not
        RUNNING thereafter.
        What to do here?
            Any initialisation you need before putting your behaviour to work.
        This initializes the variables needed to save information about the
        stop line and the traffic light.
        """
        rospy.loginfo("Approaching Intersection")
        # self.update_local_path(approach_intersection=True)
        self.start_time = rospy.get_time()
        self.stopline_detected = False
        self.stopline_distance = np.inf
        self.traffic_light_detected = False
        self.traffic_light_distance = np.inf
        self.traffic_light_status = ''
        self.virtual_stopline_distance = np.inf
        self.target_speed_pub.publish(30.0)
        self.last_virtual_distance = np.inf

    def update(self):
        """
        When is this called?
        Every time your behaviour is ticked.
        What to do here?
            - Triggering, checking, monitoring. Anything...but do not block!
            - Set a feedback message
            - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        Gets the current traffic light status and the stop line distance
        :return: py_trees.common.Status.RUNNING, if too far from intersection
                 py_trees.common.Status.SUCCESS, if stopped in front of inter-
                 section or entered the intersection
                 py_trees.common.Status.FAILURE, if no next path point can be
                 detected.
        """
        # Update Light Info
        light_status_msg = self.blackboard.get("/carla/hero/traffic_light")
        if light_status_msg is not None:
            self.traffic_light_status = light_status_msg.color
            rospy.loginfo(f"Light Status: {self.traffic_light_status}")
            self.traffic_light_distance = light_status_msg.distance
            rospy.loginfo(f"Light distance: {self.traffic_light_distance}")
        # Update stopline Info
        _dis = self.blackboard.get("/psaf/ego_vehicle/stopline_distance")
        if _dis is not None:
            self.stopline_distance = _dis.data
            rospy.loginfo(f"Stopline distance: {self.stopline_distance}")

        # calculate virtual stopline
        if self.stopline_distance != np.inf:
            self.virtual_stopline_distance = self.stopline_distance
        else:
            self.virtual_stopline_distance = self.traffic_light_distance

        # calculate speed needed for stopping
        v_stop = max(5., (self.virtual_stopline_distance / 30) ** 1.5 * 50)
        if v_stop > 30.0:
            v_stop = 30.0
        if self.virtual_stopline_distance < 3.5:
            v_stop = 0.0
        # stop when there is no or red/yellow traffic light
        if self.traffic_light_status == '' \
                or self.traffic_light_status == 'red' \
                or self.traffic_light_status == 'yellow':

            rospy.loginfo(f"slowing down: {v_stop}")
            self.target_speed_pub.publish(v_stop)

        # approach slowly when traffic light is green
        if self.traffic_light_status == 'green':
            self.target_speed_pub.publish(30)

        # get speed
        speedometer = self.blackboard.get("/carla/hero/Speed")
        speed = speedometer.speed
        if self.virtual_stopline_distance > 5.0:
            # too far
            return py_trees.common.Status.RUNNING
        elif speed < 2.0 and self.virtual_stopline_distance < 5.0:
            # stopped
            return py_trees.common.Status.SUCCESS
        elif speed > 5.0 and self.virtual_stopline_distance < 6.0 \
                and self.traffic_light_status == "green":

            # drive through intersection even if traffic light turns yellow
            return py_trees.common.Status.SUCCESS
        elif speed > 5.0 and self.virtual_stopline_distance < 3.5:
            # running over line
            return py_trees.common.Status.SUCCESS
        elif self.last_virtual_distance == self.virtual_stopline_distance \
                and self.virtual_stopline_distance < 10.0:
            # ran over line
            return py_trees.common.Status.SUCCESS

        next_lanelet_msg = self.blackboard.get("/psaf/ego_vehicle/"
                                               "next_lanelet")
        # TODO should be replaced by the next glob path point, adjust values
        if next_lanelet_msg is None:
            return py_trees.common.Status.FAILURE
        if next_lanelet_msg.distance < 12 and not next_lanelet_msg.\
                isInIntersection:
            rospy.loginfo("Leave intersection!")
            self.update_local_path(leave_intersection=True)
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """
        When is this called?
        Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting
            down
        writes a status message to the console when the behaviour terminates
        :param new_status: new state after this one is terminated
        """
        self.logger.debug(
            "  %s [Foo::terminate().terminate()][%s->%s]" % (self.name,
                                                             self.status,
                                                             new_status))


class Wait(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Wait, self).__init__(name)

    def setup(self, timeout):
        self.target_speed_pub = rospy.Publisher("/psaf/ego_vehicle/"
                                                "target_speed", Float64,
                                                queue_size=1)
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def initialise(self):
        rospy.loginfo("Wait Intersection")
        return True

    def update(self):
        light_status = self.blackboard.get("/psaf/ego_vehicle/traffic_light")
        if light_status is None:
            return py_trees.common.Status.SUCCESS
        else:
            light_status = light_status.color
        if light_status == "red" or light_status == "yellow":
            self.target_speed_pub.publish(0)
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(
            "  %s [Foo::terminate().terminate()][%s->%s]" % (self.name,
                                                             self.status,
                                                             new_status))


class Enter(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Enter, self).__init__(name)

    def setup(self, timeout):
        self.target_speed_pub = rospy.Publisher("/psaf/ego_vehicle/"
                                                "target_speed", Float64,
                                                queue_size=1)
        # rospy.wait_for_service('update_local_path')
        # self.update_local_path = rospy.ServiceProxy("update_local_path",
        # UpdateLocalPath)
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def initialise(self):
        rospy.loginfo("Enter Intersection")
        light_status = self.blackboard.get("/psaf/ego_vehicle/traffic_light")
        if light_status is None:
            self.target_speed_pub.publish(50.0)
        else:
            light_status = light_status.color
        if light_status == "":
            self.target_speed_pub.publish(10.0)
        else:
            self.target_speed_pub.publish(50.0)

    def update(self):
        next_lanelet_msg = self.blackboard.get("/psaf/ego_vehicle/"
                                               "next_lanelet")
        if next_lanelet_msg is None:
            return py_trees.common.Status.FAILURE
        if next_lanelet_msg.distance < 12 and not next_lanelet_msg.\
                isInIntersection:
            rospy.loginfo("Leave intersection!")
            self.update_local_path(leave_intersection=True)
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug(
            "  %s [Foo::terminate().terminate()][%s->%s]" % (self.name,
                                                             self.status,
                                                             new_status))


class Leave(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Leave, self).__init__(name)

    def setup(self, timeout):
        self.target_speed_pub = rospy.Publisher("/psaf/ego_vehicle/"
                                                "target_speed", Float64,
                                                queue_size=1)
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def initialise(self):
        rospy.loginfo("Leave Intersection")
        self.target_speed_pub.publish(50.0)
        return True

    def update(self):
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug(
            "  %s [Foo::terminate().terminate()][%s->%s]" % (self.name,
                                                             self.status,
                                                             new_status))
