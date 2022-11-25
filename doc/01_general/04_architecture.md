# Architecture of vehicle agent

**Summary:** This page gives an overview over the general architecture of the vehicle agent.

---

## Author

Julian Graf

## Date

24.11.2022


---
<!-- TOC -->
* [Architecture of vehicle agent](#architecture-of-vehicle-agent)
  * [Author](#author)
  * [Date](#date)
  * [Overview](#overview)
  * [Perception](#perception)
    * [Obstacle Detection and Classification](#obstacle-detection-and-classification)
    * [Lane detection](#lane-detection)
    * [Traffic Light Detection](#traffic-light-detection)
    * [Traffic Signs Detection](#traffic-signs-detection)
    * [Prediction](#prediction)
    * [Localization](#localization)
  * [Planning](#planning)
    * [Preplanning](#preplanning)
    * [Decision Making](#decision-making)
    * [Local path planning](#local-path-planning)
  * [Acting](#acting)
    * [Path following](#path-following)
    * [Velocity control](#velocity-control)
    * [Emergency](#emergency)
  * [Visualization](#visualization)
  * [Topics](#topics)
<!-- TOC -->

## Overview

The vehicle agent is split into three major components: [Perception](#Perception), [Planning](#Planning)
and [Acting](#Acting).
A separate node is responsible for the [visualization](#Visualization).

![Architecture overview](../00_assets/architecture.png) todo:better picture, maybe multiple picture

## Perception

The perception is responsible for the efficient conversion of raw sensor and map data into a useful
environment representation that can be used by the [Planning](#Planning) for further processing.

Further information can be found [here](../03_research/02_perception/Readme.md).

### Obstacle Detection and Classification

Evaluates sensor data to detect and classify objects around the ego vehicle.
Other road users and objects blocking the vehicle's path are recognized.
The node classifies objects into static and dynamic objects.
In the case of dynamic objects, an attempt is made to recognize the direction and speed of movement.

Subscriptions:

- Radar (todo)
- Lidar (todo)
- Camera (todo)

Publishes:

- Obstacles ([vision_msgs/Detection3DArray Message](http://docs.ros.org/en/api/vision_msgs/html/msg/Detection3DArray.html))

### Lane detection

Detects the lane the ego vehicle is currently in and the lanes around the ego vehicle.
This data can be used for lane keeping, to identify which lanes other road users are in,
and to plan and execute lane changes.

Subscriptions:

- Map (todo)
- Lidar (todo)
- Camera (todo)

Publishes:

- Lanes ([derived_object_msgs/LaneModels Message](http://docs.ros.org/en/kinetic/api/derived_object_msgs/html/msg/LaneModels.html))

### Traffic Light Detection

Recognizes traffic lights and what they are showing at the moment.
In particular traffic lights that are relevant for the correct traffic behavior of the ego vehicle,
are recognized early and reliably.

Subscriptions:

- Map (todo)
- Camera (todo)

Publishes:

- Traffic
  Lights ([CarlaTrafficLightStatusList.msg](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_msgs/))
  -> todo: this msg type might not be working for traffic lights not recorded on map

### Traffic Signs Detection

Recognizes traffic signs.
In particular traffic signs that are relevant for the correct traffic behavior of the ego vehicle,
are recognized early and reliably.

Subscriptions:

- Camera (todo)

Publishes:

- Traffic Lights (todo)

### Prediction

Tries to predict the movement of dynamic objects recognized in the
[Obstacle Detection and Classification](#Obstacle-Detection-and-Classification.)

Subscriptions:

- Obstacles (todo)
- Lanes (todo)

Publishes:

- Predictions (Array of custom msgs with vehicle id (uint32_t) and
  vehicle path ([nav_msgs/Path Message](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html)))

### Localization

Provides corrected accurate position, direction and speed of the ego vehicle

Subscriptions:

- Map (todo)
- IMU (todo)
- Speedometer (todo)
- GNSS (todo)

Publishes:

- Position and Pose ([nav_msgs/Odometry Message](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))

## Planning

The planning uses the data from the [Perception](#Perception) to find a path on which the ego vehicle can safely reach
its destination

Further information can be found [here](../03_research/03_planning/Readme.md).

### Preplanning

Uses information from the map and the path specified by CARLA to find a first concrete path to the next intermediate
point.
Information from Obstacle Detection, Prediction etc. is not yet taken into account.

Subscriptions:

- Map (todo)
- Navigation (todo)
- Odometrie (todo)

Publishes:

- Provisional Path ([nav_msgs/Path Message](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))

### Decision Making

checks whether the path from [Preplanning](#Preplanning) actually can be taken.
If the data from the [Perception](#Perception) indicates that the path needs to be adjusted,
this node decides which actions to take.
Based on this decision, the [Local path planning](#Local-path-planning) plans a new path accordingly.

Subscriptions:

- Map (todo)
- Navigation (todo)
- Odometrie (todo)
- Provisional Path (todo)
- all Data from Perception (todo)

Publishes:

- Decision (string)

### Local path planning

Translates the decisions made by the [Decision Making](#Decision-Making) into a concrete path.
Can publish the distance to the vehicle in front to use the [Path following](#Path-folloing)'s adaptive cruise control.

Subscriptions:

- Map (todo)
- Odometrie (todo)
- Provisional Path (todo)
- all Data from Perception (todo)

Publishes:

- Path ([nav_msgs/Path Message](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))
- velocity (uint8_t)
- distance to vehicle to follow (uint8_t)

## Acting

The job of this component is to translate the trajectory planned by the [Planning](#Planning) component into
steering controls for the vehicle.

Further information can be found [here](../03_research/01_acting/Readme.md).

### Path following

Calculates steering angles that keep the ego vehicle on the path given by
the [Local path planning](#Local-lath-planning).

Subscriptions:

- Path ([nav_msgs/Path Message](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))

Publishes:

- steering angle for rosbridge (todo)

### Velocity control

Calculates velocitiy inputs to drive the velocity given by the [Local path planning](#Local-lath-planning).
If the node is given the distance to a car to follow, it reduces the velocity of the ego vehicle to hold a reasonable
distance to the vehicle in front.

Subscriptions:

- velocity (uint8_t)
- distance to vehicle to follow (uint8_t)
- Path ([nav_msgs/Path Message](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html))

Publishes:

- velocity for rosbridge (todo)

### Emergency

Reads data from all nodes to detect emergency situations.
Initiates emergency braking in the event of an emergency and notifies all notes that an emergency stop has occurred.

Subscriptions:

- whatever it needs (todo)

Publishes:

- velocity for rosbridge (todo)
- steering angle for rosbridge (todo)
- emergency msg (bool)

## Visualization

Visualizes outputs of certain nodes to provide a basis for debugging.

## Topics
todo

| Topic | Description | Msg type |
|-------|-------------|----------|
| todo  |             |          |
