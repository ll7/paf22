#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import py_trees
# from std_msgs.msg import Float64
# from nav_msgs.msg import Odometry
# import numpy as np
import math

"""
Source: https://github.com/ll7/psaf2
"""


class IntersectionAhead(py_trees.behaviour.Behaviour):
    """
    This behaviour checkes wheather there is an intersection in front of the
    ego vehicle or not and triggers the rest of the decision tree handling the
     intersection.
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
        super(IntersectionAhead, self).__init__(name)

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
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def initialise(self):
        """
        When is this called?
            The first time your behaviour is ticked and anytime the status is
            not RUNNING thereafter.
        What to do here?
            Any initialisation you need before putting your behaviour to work.
        This initializes the variables needed to save information about the
        stop line.
        """
        self.dist = 0

    def update(self):
        """
        When is this called?
        Every time your behaviour is ticked.
        What to do here?
            - Triggering, checking, monitoring. Anything...but do not block!
            - Set a feedback message
            - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        Gets the current distance to the next intersection.
        :return: py_trees.common.Status.SUCCESS, if the vehicle is within range
                    of the intersection
                 py_trees.common.Status.FAILURE, if we are too far away from
                 the intersection
        """
        # TODO change this part to the actual source of intersection detection
        bb = self.blackboard.get("/paf/hero/stop_sign")
        if bb is None:
            return py_trees.common.Status.FAILURE
        else:
            self.dist = bb.distance
        if self.dist < 30:
            return py_trees.common.Status.SUCCESS
        else:
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
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


class RoundaboutAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(RoundaboutAhead, self).__init__(name)

    def setup(self, timeout):
        self.Roundabout = False
        return True

    def initialise(self):
        self.blackboard = py_trees.blackboard.Blackboard()
        self.dist = 0

    def update(self):
        bb = self.blackboard.get("/psaf/ego_vehicle/distance_next_roundabout")
        self.odo = self.blackboard.get("/carla/ego_vehicle/odometry")
        if bb is None:
            return py_trees.common.Status.FAILURE
        else:
            dist_x = bb.entry_point.x - self.odo.pose.pose.position.x
            dist_y = bb.entry_point.y - self.odo.pose.pose.position.y
            dist = math.sqrt(dist_x ** 2 + dist_y ** 2)
            self.dist = dist
        if self.dist < 30:
            rospy.loginfo("approaching roundabout")
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


class StopAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(StopAhead, self).__init__(name)

    def setup(self, timeout):
        self.Stop = False
        return True

    def initialise(self):
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        if self.Stop:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


class MultiLane(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(MultiLane, self).__init__(name)

    def setup(self, timeout):
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def initialise(self):
        pass

    def update(self):
        bb = self.blackboard.get("/psaf/ego_vehicle/lane_status")
        if bb is None:
            return py_trees.common.Status.FAILURE
        if bb.isMultiLane:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


class SingleLineDotted(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(SingleLineDotted, self).__init__(name)

    def setup(self, timeout):
        self.Success = False
        return True

    def initialise(self):
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        if self.Success:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


class RightLaneAvailable(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(RightLaneAvailable, self).__init__(name)

    def setup(self, timeout):
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def initialise(self):
        pass

    def update(self):
        bb = self.blackboard.get("/psaf/ego_vehicle/lane_status")
        if bb is None:
            return py_trees.common.Status.FAILURE
        if bb.rightLaneId != -1:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


class LeftLaneAvailable(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(LeftLaneAvailable, self).__init__(name)

    def setup(self, timeout):
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def initialise(self):
        pass

    def update(self):
        bb = self.blackboard.get("/psaf/ego_vehicle/lane_status")
        if bb is None:
            return py_trees.common.Status.FAILURE
        if bb.leftLaneId != -1:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s ]" %
                          (self.name, self.status, new_status))
