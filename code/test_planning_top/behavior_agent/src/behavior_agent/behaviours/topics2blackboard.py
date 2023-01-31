#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees
import py_trees_ros

from std_msgs.msg import Float64, String, Bool, Int32, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

from geometry_msgs.msg import PoseWithCovarianceStamped


def create_node(role_name):
    topics = [
        {'name': f"/carla/{role_name}/odometry", 'msg': Odometry,
         'clearing-policy': py_trees.common.ClearingPolicy.NEVER},
        {'name': f"/psaf/{role_name}/target_speed", 'msg': Float64,
         'clearing-policy': py_trees.common.ClearingPolicy.NEVER},
        {'name': f"/psaf/{role_name}/obstacle", 'msg': String,
         'clearing-policy': py_trees.common.ClearingPolicy.NEVER},
        {'name': f"/psaf/{role_name}/bt/condition/slowed_by_car_in_front", 'msg': Bool,
         'clearing-policy': py_trees.common.ClearingPolicy.NEVER},
        {'name': f"/psaf/{role_name}/stopline_distance", 'msg': Float64,
         'clearing-policy': py_trees.common.ClearingPolicy.NEVER},
        {'name': f"/psaf/{role_name}/distance_exit_roundabout", 'msg': Point,
         'clearing-policy': py_trees.common.ClearingPolicy.NEVER},
        {'name': f"/psaf/{role_name}/obstacle_on_left_lane", 'msg': Float64,
         'clearing-policy': py_trees.common.ClearingPolicy.NEVER},
        {'name': f"/psaf/{role_name}/obstacle_on_right_lane", 'msg': Float64,
         'clearing-policy': py_trees.common.ClearingPolicy.NEVER},
        {'name': "/initialpose", 'msg': PoseWithCovarianceStamped,
         'clearing-policy': py_trees.common.ClearingPolicy.NEVER},
        {'name': f"/carla/{role_name}/initialpose", 'msg': PoseWithCovarianceStamped,
         'clearing-policy': py_trees.common.ClearingPolicy.NEVER},
        {'name': f"/psaf/{role_name}/first_lanelet_roundabout", 'msg': Int32,
         'clearing-policy': py_trees.common.ClearingPolicy.NEVER},
        {'name': f"/carla/{role_name}/max_velocity", 'msg': Float32,
         'clearing-policy': py_trees.common.ClearingPolicy.NEVER}
    ]

    topics2blackboard = py_trees.composites.Parallel("Topics to Blackboard")
    for topic in topics:
        topics2blackboard.add_child(
            py_trees_ros.subscribers.ToBlackboard(name=topic['name'],
                                                  topic_name=topic['name'],
                                                  topic_type=topic['msg'],
                                                  blackboard_variables={topic['name']: None},
                                                  clearing_policy=topic['clearing-policy']))
    return topics2blackboard
