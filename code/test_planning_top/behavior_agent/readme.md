[//]: # ()
[//]: # (""")

[//]: # (Source: https://github.com/ll7/psaf2)

[//]: # (""")

# behaviour_agent

## About

This Package implements a behaviour agent for our autonomous car using __Behaviour Trees__. It uses the _py_trees_ Framework, that works well with ROS. All the dependencies for that library should be included in the installation instructions (wiki).
For visualization at runtime you might want to also install this [rqt-Plugin](https://wiki.ros.org/rqt_py_trees).

## Our behaviour tree

The following section describes the behaviour tree we use for normal driving with all traffic rules. In the actual implementation this is part of a bigger tree, that handles things like writing topics to the blackboard, switching modes and respawning.
The following description is not complete, it just contains the most common behaviours and subtrees. For a complete description have a look at the [tree-description](https://github.com/ll7/psaf2/blob/main/documentation/behaviour_agent/behaviortree.xml)
(you can open it with [Groot](https://github.com/BehaviorTree/Groot)) and the [bt-specs](https://github.com/ll7/psaf2/blob/main/documentation/BTSpecs.md).
Note that we didnt actually implement all of the behaviours from this design, due to time limitations.

### Legend

The following Notation is used in this documentation:
![BT Legend](https://github.com/ll7/psaf2/blob/main/documentation/behaviour_agent/bt-legend.svg)

### Big Picture

![BT Big Picture](https://github.com/ll7/psaf2/blob/main/documentation/behaviour_agent/bt_big_picture.svg)
This top-level tree consists mainly of sub-trees that are explained below. If none of the subtrees fit the current situation, the behaviour_agent goes into _Cruising_-behaviour, where it just follows the Path at an appropriate speed.

### Intersection

![BT Intersection](https://github.com/ll7/psaf2/blob/main/documentation/behaviour_agent/bt-intersection.svg)

If there is a Intersection coming up the agent executes the following sequence of behaviours:

* Approach Intersection

    Slows down, gets into the right lane for turning and stops at line
* Wait at Intersection

    Waits for traffic lights or higher priority traffic
* Enter Intersection

    Enters the intersection and stops again, if there is higher priority oncoming traffic
* Leave Intersection

    Leaves the intersection in the right direction

### Roundabout

![BT Roundabout](https://github.com/ll7/psaf2/blob/main/documentation/behaviour_agent/bt-roundabout.svg)
This subtree is basically identical to the intersection-subtree. The implementation of the behaviours varies a bit though.

### Overtaking

The Overtaking subtree is quite big to accommodate for different overtaking scenarios.
Please have a look at the [tree-description](https://github.com/ll7/psaf2/blob/main/documentation/behaviour_agent/behaviortree.xml) and the [bt-specs](https://github.com/ll7/psaf2/blob/main/documentation/BTSpecs.md) for a more detailed description. The Multi-Lane Overtaking Subtree looks like this:
![BT Overtaking](https://github.com/ll7/psaf2/blob/main/documentation/behaviour_agent/bt-overtaking.svg)

* Multi Lane?

    Checks the map data: does the current road have more than one lane?
* Left Lane available?

    Checks the map data: is there a lane available to the left of the ego vehicle?
* Wait for Left Lane free

    Waits for the left lane to be free. This has a timeout.
* Switch Lane Left

    Triggers a Laneswitch to the left by calling the local planner

### Right-Hand Driving

This subtree makes the ego vehicle switch back to the right lane, if the road ahead is free enough. It is quite similar to the Multi-Lane Overtaking Subtree, just with reversed directions.

![BT Right Hand](https://github.com/ll7/psaf2/blob/main/documentation/behaviour_agent/bt-right-hand.svg)

## Developing guide

### Tree Definition

The tree is defined in the `grow_a_tree()`-function inside `src/behavior_agent/behavior_tree.py`, which is also the main node. It can be visualized using an [rqt-Plugin](https://wiki.ros.org/rqt_py_trees). This is also the Place to change the execution rate of the tree:

``` python
...
behaviour_tree.tick_tock(500)
...
```

### Behaviours

_Behaviours_ are implemented in the `src/behavior_agent/behaviours/` directory. All the behaviours used in the current Version of the tree are contained as skeletons.

#### Blackboard

To deal with the asynchronity of ROS, all the Topics this Tree subscribes to, should be written to the Blackboard at the beginning of each tick. I wrote a Node, that automates this Task. Just add your Node to the list in `src/behavior_agent/behaviours/topics2blackboard.py`:

``` python
...
topics =[
    {'name':f"/carla/{role_name}/odometry", 'msg':Odometry, 'clearing-policy': py_trees.common.ClearingPolicy.NEVER},
    ...
    ]
...
```

After that you can access them from everywhere in your Behaviour-Code using:

``` python
...
self.blackboard = py_trees.blackboard.Blackboard()
...
odo = self.blackboard.get("/carla/ego_vehicle/odometry")
...
```

Note that you still need to resolve the data-fields of the message (i.e. `blackboardmessage.data` for a `Float64`).

### Guidelines

When implementing new behaviours you should adhere to the following guidelines:

#### Non-Blocking

You should avoid doing complicated calculations inside the behaviours. Use asynchronous Callbacks instead, and return ```RUNNING``` while another Node does the computing.

Generally Conditions should never return ```RUNNING``` and Action-Behaviours should only return ```FAILURE``` in special cases.

#### Functions

Behaviours generally provide five Functions (you can define more of course). Short Explanation when they get called and how to use them:

##### `__init__()`

You should probably never use this.

##### `setup()`

Gets called whenever the tree gets set up for the first time. Use this to setup local variables that dont need to change, like ```self.blackboard = py_trees.blackboard.Blackboard()``` or Middleware like ROS-Publishers (Subscribers should be setup using the method mentioned above).

##### `initialise()`

Gets called everytime the Behaviour is entered for a new execution. Add Code that only needs to called once at the beginning of a behaviour (i.e. publishing a new target speed).

##### `update()`

Main Function of a behaviour, that gets called everytime the behaviour is ticked. Here you need to return ```SUCCESS```, ```RUNNING``` or ```FAILURE```.

##### `terminate()`

This gets called, whenever a behaviour is cancelled by a higher priority branch. Use to terminate Middleware-Connections or Asynchronous Calculations, whose Results are not needed anymore.

## Authors

Valentin Höpfner
