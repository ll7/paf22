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
    def __init__(self, name):
        super(IntersectionAhead, self).__init__(name)

    def setup(self, timeout):
        self.currentstart = 0.0
        self.blackboard = py_trees.blackboard.Blackboard()
        self.lastdist = 0.0
        return True

    def initialise(self):
        self.dist = 0

    def update(self):
        # TODO Write data field to blackboard directly
        bb = self.blackboard.get("/psaf/ego_vehicle/next_lanelet")
        if bb is None:
            return py_trees.common.Status.FAILURE
        else:
            self.dist = bb.distance
        if self.dist < 30 and bb.isInIntersection:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
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
    """
    This behavior decides if the road the agent is currently on, has more than
    one lane for the driving direction. This could be used to change lanes to
    the right to perhaps evade an emergency vehicle.
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
        super(MultiLane, self).__init__(name)

    def setup(self, timeout):
        """
        Delayed one-time initialisation that would otherwise interfere with
        offline rendering of this behaviour in a tree to dot graph or
        validation of the behaviour's configuration.

        :param timeout: an initial timeout to see if the tree generation is
        successful
        :return: True, as there is nothing to set up.
        """
        return True

    def initialise(self):
        """
        When is this called?
        The first time your behaviour is ticked and anytime the status is not
        RUNNING thereafter.

        What to do here?
            Any initialisation you need before putting your behaviour to work.

        This initializes the blackboard to be able to access data written to it
        by the ROS topics.
        """
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        """
        When is this called?
        Every time your behaviour is ticked.

        What to do here?
            - Triggering, checking, monitoring. Anything...but do not block!
            - Set a feedback message
            - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]

        This part checks if the agent is on a multi-lane and corresponding
        overtaking behavior should be triggered. If we are not on a multi-lane
        it is check if overtaking on a single lane is possible. Otherwise, the
        overtaking process will be canceled.

        :return: py_trees.common.Status.SUCCESS, if the agent is on a multi-
                 lane
                 py_trees.common.Status.FAILURE, if the agent is not on a
                 multi-lane
        """
        bb = self.blackboard.get("/paf/hero/lane_status")
        if bb is None:
            return py_trees.common.Status.FAILURE
        if bb.isMultiLane:
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
        """
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


class SingleLineDotted(py_trees.behaviour.Behaviour):
    """
    This behavior checks if it is allowed to switch lanes one a single lane
    street.
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
        super(SingleLineDotted, self).__init__(name)

    def setup(self, timeout):
        """
        Delayed one-time initialisation that would otherwise interfere with
        offline rendering of this behaviour in a tree to dot graph or
        validation of the behaviour's configuration.

        :param timeout: an initial timeout to see if the tree generation is
        successful
        :return: True, as there is nothing to set up.
        """
        self.Success = False
        return True

    def initialise(self):
        """
        When is this called?
        The first time your behaviour is ticked and anytime the status is not
        RUNNING thereafter.

        What to do here?
            Any initialisation you need before putting your behaviour to work.

        This initializes the blackboard to be able to access data written to it
        by the ROS topics.
        """
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        """
        When is this called?
        Every time your behaviour is ticked.

        What to do here?
            - Triggering, checking, monitoring. Anything...but do not block!
            - Set a feedback message
            - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]

        Right now, there is no way for us to detect a single dotted line, so it
        is assumed that we are on one.

        :return: py_trees.common.Status.SUCCESS, if the agent is on a single
                 dotted line
                 py_trees.common.Status.FAILURE, if the agent is not on a
                 single dotted line
        """
        if self.Success:
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
        """
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


class RightLaneAvailable(py_trees.behaviour.Behaviour):
    """
    This behavior checks if there is a lane to the right of the agent it could
    change to.
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
        super(RightLaneAvailable, self).__init__(name)

    def setup(self, timeout):
        """
        Delayed one-time initialisation that would otherwise interfere with
        offline rendering of this behaviour in a tree to dot graph or
        validation of the behaviour's configuration.

        :param timeout: an initial timeout to see if the tree generation is
        successful
        :return: True, as there is nothing to set up.
        """
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def initialise(self):
        """
        When is this called?
        The first time your behaviour is ticked and anytime the status is not
        RUNNING thereafter.

        What to do here?
           Any initialisation you need before putting your behaviour to work.

        This initializes the blackboard to be able to access data written to it
        by the ROS topics.
        """
        pass

    def update(self):
        """
        When is this called?
        Every time your behaviour is ticked.

        What to do here?
           - Triggering, checking, monitoring. Anything...but do not block!
           - Set a feedback message
           - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]

        This part checks if there is a lane to the right of the agent.

        :return: py_trees.common.Status.SUCCESS, if there is a lane to the
                 right of the agent
                 py_trees.common.Status.FAILURE, if there is no lane to the
                 right of the agent
        """
        bb = self.blackboard.get("/paf/hero/lane_status")
        if bb is None:
            return py_trees.common.Status.FAILURE
        if bb.rightLaneId != -1:
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
        """
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


class LeftLaneAvailable(py_trees.behaviour.Behaviour):
    """
    On a multi-lane, this behavior checks if there is a lane to the left of the
    agent it could change to, to overtake.
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
        super(LeftLaneAvailable, self).__init__(name)

    def setup(self, timeout):
        """
        Delayed one-time initialisation that would otherwise interfere with
        offline rendering of this behaviour in a tree to dot graph or
        validation of the behaviour's configuration.

        :param timeout: an initial timeout to see if the tree generation is
        successful
        :return: True, as there is nothing to set up.
        """

        return True

    def initialise(self):
        """
        When is this called?
        The first time your behaviour is ticked and anytime the status is not
        RUNNING thereafter.

        What to do here?
            Any initialisation you need before putting your behaviour to work.

        This initializes the blackboard to be able to access data written to it
        by the ROS topics.
        """
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        """
        When is this called?
        Every time your behaviour is ticked.

        What to do here?
           - Triggering, checking, monitoring. Anything...but do not block!
           - Set a feedback message
           - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]

        This part checks if there is a lane to the left of the agent and the
        rest of the overtaking can take place.

        :return: py_trees.common.Status.SUCCESS, if there is a lane to the left
                 of the agent
                 py_trees.common.Status.FAILURE, if there is no lane to the
                 left of the agent
        """
        # TODO this information could be retrieved from the openDrive map, not
        # sure if that will happen though due to the limited time left
        bb = self.blackboard.get("/paf/hero/lane_status")
        if bb is None:
            return py_trees.common.Status.FAILURE
        if bb.leftLaneId != -1:
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
        """
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s ]" %
                          (self.name, self.status, new_status))
