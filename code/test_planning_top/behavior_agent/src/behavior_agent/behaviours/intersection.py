import py_trees
import numpy as np
from std_msgs.msg import Float32
# from nav_msgs.msg import Odometry
# from custom_carla_msgs.srv import UpdateLocalPath

import rospy

"""
Source: https://github.com/ll7/psaf2
"""


def convert_to_ms(speed):
    return speed / 3.6


class Approach(py_trees.behaviour.Behaviour):
    """
    This behaviour is executed when the ego vehicle is in close proximity of
    an intersection and behaviours.road_features.intersection_ahead is
    triggered. It than handles the approaching the intersection, slowing the
    vehicle down appropriately.
    """
    def __init__(self, name):
        """
        Minimal one-time initialisation. Other one-time initialisation
        requirements should be met via the setup() method.
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
        self.target_speed_pub = rospy.Publisher("/carla/hero/"
                                                "max_tree_velocity",
                                                Float32, queue_size=1)
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
        self.target_speed_pub.publish(convert_to_ms(30.0))
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
        light_status_msg = self.blackboard.get("/paf/hero/traffic_light")
        if light_status_msg is not None:
            self.traffic_light_status = light_status_msg.color
            rospy.loginfo(f"Light Status: {self.traffic_light_status}")
            self.traffic_light_distance = light_status_msg.distance
            rospy.loginfo(f"Light distance: {self.traffic_light_distance}")
        # Update stopline Info
        _dis = self.blackboard.get("/paf/hero/stopline_distance")
        if _dis is not None:
            self.stopline_distance = _dis.data
            rospy.loginfo(f"Stopline distance: {self.stopline_distance}")

        # calculate virtual stopline
        if self.stopline_distance != np.inf:
            self.virtual_stopline_distance = self.stopline_distance
        else:
            self.virtual_stopline_distance = self.traffic_light_distance

        # calculate speed needed for stopping
        v_stop = max(convert_to_ms(5.),
                     convert_to_ms((self.virtual_stopline_distance / 30) ** 1.5
                                   * 50))
        if v_stop > convert_to_ms(50.0):
            v_stop = convert_to_ms(30.0)
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
            self.target_speed_pub.publish(convert_to_ms(30))

        # get speed
        speedometer = self.blackboard.get("/carla/hero/Speed")
        if speedometer is not None:
            speed = speedometer.speed
        else:
            rospy.logwarn("no speedometer connected")
        if self.virtual_stopline_distance > 5.0:
            # too far
            return py_trees.common.Status.RUNNING
        elif speed < convert_to_ms(2.0) and \
                self.virtual_stopline_distance < 5.0:
            # stopped
            return py_trees.common.Status.SUCCESS
        elif speed > convert_to_ms(5.0) and \
                self.virtual_stopline_distance < 6.0 and \
                self.traffic_light_status == "green":

            # drive through intersection even if traffic light turns yellow
            return py_trees.common.Status.SUCCESS
        elif speed > convert_to_ms(5.0) and \
                self.virtual_stopline_distance < 3.5:
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
    """
    This behavior handles the waiting in front of the stop line at the inter-
    section until there either is no traffic light or the traffic light is
    green.
    """
    def __init__(self, name):
        """
        Minimal one-time initialisation. Other one-time initialisation
        requirements should be met via the setup() method.
         :param name: name of the behaviour
        """
        super(Wait, self).__init__(name)

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
        self.target_speed_pub = rospy.Publisher("/carla/hero/"
                                                "max_tree_velocity", Float32,
                                                queue_size=1)
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def initialise(self):
        """
        When is this called?
            The first time your behaviour is ticked and anytime the status is
            not RUNNING thereafter.
        What to do here?
            Any initialisation you need before putting your behaviour to work.
        This just prints a state status message.
        """
        rospy.loginfo("Wait Intersection")
        return True

    def update(self):
        """
        When is this called?
            Every time your behaviour is ticked.
        What to do here?
           - Triggering, checking, monitoring. Anything...but do not block!
           - Set a feedback message
           - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        Waits in front of the intersection until there is a green light or no
        traffic light at all.
        :return: py_trees.common.Status.RUNNING, while traffic light is yellow
                 or red
                 py_trees.common.Status.SUCCESS, if the traffic light switched
                 to green or no traffic light is detected
        """
        light_status_msg = self.blackboard.get("/paf/hero/traffic_light")
        if light_status_msg is None:
            rospy.loginfo("No traffic light detected")
            return py_trees.common.Status.SUCCESS
        else:
            traffic_light_status = light_status_msg.color
        if traffic_light_status == "red" or traffic_light_status == "yellow":
            rospy.loginfo(f"Light Status: {traffic_light_status}")
            self.target_speed_pub.publish(0)
            return py_trees.common.Status.RUNNING
        else:
            rospy.loginfo(f"Light Status: {traffic_light_status}")
            return py_trees.common.Status.SUCCESS

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


class Enter(py_trees.behaviour.Behaviour):
    """
    This behavior handles the driving through an intersection, it initially
    sets a speed and finishes if the ego vehicle is close to the end of the
    intersection.
    """
    def __init__(self, name):
        """
        Minimal one-time initialisation. Other one-time initialisation
        requirements should be met via the setup() method.
        :param name: name of the behaviour
        """
        super(Enter, self).__init__(name)

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
        self.target_speed_pub = rospy.Publisher("/carla/hero/"
                                                "max_tree_velocity", Float32,
                                                queue_size=1)
        # rospy.wait_for_service('update_local_path')
        # self.update_local_path = rospy.ServiceProxy("update_local_path",
        # UpdateLocalPath)
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def initialise(self):
        """
        When is this called?
            The first time your behaviour is ticked and anytime the status is
            not RUNNING thereafter.
        What to do here?
            Any initialisation you need before putting your behaviour to work.
        This prints a state status message and changes the driving speed for
        the intersection.
        """
        rospy.loginfo("Enter Intersection")
        light_status_msg = self.blackboard.get("/paf/hero/traffic_light")
        if light_status_msg is None:
            self.target_speed_pub.publish(convert_to_ms(50.0))
        else:
            traffic_light_status = light_status_msg.color
        if traffic_light_status == "":
            self.target_speed_pub.publish(convert_to_ms(10.0))
        else:
            rospy.loginfo(f"Light Status: {traffic_light_status}")
            self.target_speed_pub.publish(convert_to_ms(50.0))

    def update(self):
        """
        When is this called?
            Every time your behaviour is ticked.
        What to do here?
           - Triggering, checking, monitoring. Anything...but do not block!
           - Set a feedback message
           - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        Continues driving through the intersection until the vehicle gets
        close enough to the next global way point.
        :return: py_trees.common.Status.RUNNING, if too far from intersection
                 py_trees.common.Status.SUCCESS, if stopped in front of inter-
                 section or entered the intersection
                 py_trees.common.Status.FAILURE, if no next path point can be
                 detected.
        """
        # TODO this part needs to be refurbished when we have a publisher for
        # the next global way point
        # next_lanelet_msg = self.blackboard.get("/psaf/ego_vehicle/"
        #                                        "next_lanelet")

        rospy.loginfo("Through intersection")
        return py_trees.common.Status.SUCCESS

        # if next_lanelet_msg is None:
        #     return py_trees.common.Status.FAILURE
        # if next_lanelet_msg.distance < 12 and not next_lanelet_msg.\
        #         isInIntersection:
        #     rospy.loginfo("Leave intersection!")
        #     self.update_local_path(leave_intersection=True)
        #     return py_trees.common.Status.SUCCESS
        # else:
        #     return py_trees.common.Status.RUNNING

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


class Leave(py_trees.behaviour.Behaviour):
    """
    This behaviour defines the leaf of this subtree, if this behavior is
    reached, the vehicle left the intersection.
    """
    def __init__(self, name):
        """
        Minimal one-time initialisation. Other one-time initialisation
        requirements should be met via the setup() method.
        :param name: name of the behaviour
        """
        super(Leave, self).__init__(name)

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
        self.target_speed_pub = rospy.Publisher("/carla/hero/"
                                                "max_tree_velocity", Float32,
                                                queue_size=1)
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def initialise(self):
        """
        When is this called?
            The first time your behaviour is ticked and anytime the status is
            not RUNNING thereafter.
        What to do here?
            Any initialisation you need before putting your behaviour to work.
        This prints a state status message and changes the driving speed to
        the street speed limit.
        """
        rospy.loginfo("Leave Intersection")
        street_speed_msg = self.blackboard.get("/carla/hero/street_limit")
        if street_speed_msg is not None:
            self.target_speed_pub.publish(street_speed_msg.speed)
        return True

    def update(self):
        """
        When is this called?
            Every time your behaviour is ticked.
        What to do here?
           - Triggering, checking, monitoring. Anything...but do not block!
           - Set a feedback message
           - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        Abort this subtree
        :return: py_trees.common.Status.FAILURE, to exit this subtree
        """
        return py_trees.common.Status.FAILURE

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
