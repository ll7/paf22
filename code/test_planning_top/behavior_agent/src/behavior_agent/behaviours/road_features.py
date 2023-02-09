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
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))
