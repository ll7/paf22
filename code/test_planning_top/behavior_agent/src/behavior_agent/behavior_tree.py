#!/usr/bin/env python

import functools
import behavior_agent
import py_trees
from py_trees.behaviours import Running
import py_trees_ros
import py_trees.console as console
import rospy
import sys
from behavior_agent import behaviours
from py_trees.composites import Parallel, Selector, Sequence
from py_trees.decorators import Inverter


def grow_a_tree(role_name):

    rules = Parallel(
        "Rules",
        children=[
            Selector
            ("Priorities",
                children=[
                    behaviours.meta.RespawnOrFinish("Respawn or Finish?"),
                    Inverter(
                        Selector("Avoid Collisions",
                                 children=[
                                     behaviours.avoid_collisions.
                                     NoObstacleAhead("No Obstacle Ahead?"),
                                     Selector("Collision Avoidance Action",
                                              children=[
                                                  behaviours.avoid_collisions.ReplanAroundObstacles
                                                  ("Replan around Obstacles"),
                                                  behaviours.avoid_collisions.EmergencyBrake
                                                  ("Emergency Brake")
                                              ])
                                 ])),
                    Selector("Road Features",
                             children=[
                                 Sequence("Intersection",
                                          children=[
                                              behaviours.road_features.IntersectionAhead
                                              ("Intersection Ahead"),
                                              Sequence("Intersection Actions",
                                                       children=[
                                                           behaviours.intersection.Approach
                                                           ("Approach Intersection"),
                                                           behaviours.intersection.Wait
                                                           ("Wait Intersection"),
                                                           behaviours.intersection.Enter
                                                           ("Enter Intersection"),
                                                           behaviours.intersection.Leave
                                                           ("Leave Intersection")
                                                       ])
                                          ]),
                                 Sequence("Stop",
                                          children=[
                                              behaviours.road_features.StopAhead("Stop Ahead"),
                                              Sequence("Stop Actions",
                                                       children=[
                                                           behaviours.stop.Approach
                                                           ("Approach Stop"),
                                                           behaviours.roundabout.Approach
                                                           ("Wait Stop"),
                                                           behaviours.roundabout.Approach
                                                           ("Leave Stop")
                                                       ])
                                          ])
                             ]),
                    Selector("Laneswitching", children=[
                        Inverter(Selector("Overtaking", children=[
                            behaviours.traffic_objects.NotSlowedByCarInFront
                            ("Not Slowed By Car in Front?"),
                            Selector("Number of Lanes", children=[
                                Sequence("Multi Lane", children=[
                                    behaviours.road_features.MultiLane("Multi Lane?"),
                                    behaviours.road_features.LeftLaneAvailable
                                    ("Left Lane Available?"),
                                    behaviours.traffic_objects.WaitLeftLaneFree
                                    ("Wait for Left Lane Free"),
                                    behaviours.maneuvers.SwitchLaneLeft("Switch Lane Left")
                                ]),
                                Sequence("Single Lane", children=[
                                    behaviours.road_features.SingleLineDotted
                                    ("Single Lane with dotted Line?"),
                                    behaviours.traffic_objects.WaitLeftLaneFree
                                    ("Wait for Left Lane Free"),
                                    behaviours.maneuvers.SwitchLaneLeft
                                    ("Switch Lane Left"),
                                    Selector("Driving on Left Side", children=[
                                        Sequence("Overtake", children=[
                                            behaviours.traffic_objects.OvertakingPossible
                                            ("Overtaking Possible?"),
                                            behaviours.maneuvers.Overtake("Overtake"),
                                            behaviours.maneuvers.SwitchLaneRight
                                            ("Switch Lane Right")
                                        ]),
                                        behaviours.maneuvers.SwitchLaneRight("Switch Lane Right")
                                    ])
                                ])
                            ]),
                            Running("Can't Overtake")
                        ])),
                        Sequence("Back to Right Lane", children=[
                            behaviours.road_features.RightLaneAvailable("Right Lane Available"),
                            behaviours.traffic_objects.NotSlowedByCarInFrontRight
                            ("Not Slowed By Car in Front Right?"),
                            behaviours.traffic_objects.
                            WaitRightLaneFree("Wait for Right Lane Free"),
                            behaviours.maneuvers.SwitchLaneRight("Switch Lane Right")
                        ])
                    ]),
                    behaviours.maneuvers.Cruise("Cruise")
                ])
        ])

    metarules = Sequence("Meta", children=[behaviours.meta.Start("Start"), rules,
                                           behaviours.meta.End("End")])
    root = Parallel("Root", children=[
        behaviours.topics2blackboard.create_node(role_name),
        Sequence("Rules", children=[
            behaviours.meta.Rules("Rules?"),
            metarules
        ]),
        Running("Idle")
    ])
    return root


def shutdown(behaviour_tree):
    behaviour_tree.interrupt()


def main():
    """
    Entry point for the demo script.
    """
    rospy.init_node('behavior_tree', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    root = grow_a_tree(role_name)
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))

    if not behaviour_tree.setup(timeout=15):
        rospy.loginfo("Tree Setup failed")
        sys.exit(1)
    rospy.loginfo("tree setup worked")
    r = rospy.Rate(5)
    while not rospy.is_shutdown():
        behaviour_tree.tick()
        try:
            r.sleep()
        except rospy.ROSInterruptException:
            pass


if __name__ == "__main__":
    main()
