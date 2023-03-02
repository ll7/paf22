#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees
# import rospy
# import math
# from nav_msgs.msg import Odometry

"""
Source: https://github.com/ll7/psaf2
"""


class NoObstacleAhead(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(NoObstacleAhead, self).__init__(name)

    def setup(self, timeout):
        self.logger.debug("  %s [Foo::setup()]" % self.name)
        return True

    def initialise(self):
        self.blackboard = py_trees.blackboard.Blackboard()

        self.logger.debug("  %s [Foo::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [Foo::update()]" % self.name)
        obstacle = self.blackboard.get("/psaf/ego_vehicle/obstacle")
        if obstacle is not None:
            print(obstacle.data)
        if (obstacle is None) or (obstacle.data == "Fehlalarm"):
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]"
                          % (self.name, self.status, new_status))


class ReplanAroundObstacles(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(ReplanAroundObstacles, self).__init__(name)

    def setup(self, timeout):

        self.logger.debug("  %s [Foo::setup()]" % self.name)
        return True

    def initialise(self):
        self.logger.debug("  %s [Foo::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [Foo::update()]" % self.name)

        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


class EmergencyBrake(py_trees.behaviour.Behaviour):
    def __init__(self, name):

        super(EmergencyBrake, self).__init__(name)

    def setup(self, timeout):
        return True

    def initialise(self):
        self.logger.debug("  %s [Foo::initialise()]" % self.name)

    def update(self):

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))
