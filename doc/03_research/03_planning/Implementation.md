# Coding style guidelines

**Summary:** 
---

## Author

Simon Erlbacher

## Date

24.11.2022

## Prerequisite


---
<!-- TOC -->
* [Planning Implementation Details](#coding-style-guidelines)
  * [Author](#author)
  * [Date](#date)
  * [Prerequisite](#prerequisite)
  * [Visualization](#visualization)
  * [Preplanning](#preplanning)
  * [Local Path Planning](#local-path-planning)
  * [Decision Making](#decision-making)
  * [Next steps](#next-steps)
* [Sources](#sources)
<!-- TOC -->

---
## **Visualization** 

![Implementation](./doc/00_assets/Planning_Implemenation.png)

---
## **Preplanning**
  
The Preplanning receives the next target point from the Carla Leaderboard. Also it reads the Open Street Format. The commonroad route planner from TUM can be used to calculate the preferred Lanelet model. The Open Street Format will be transformed in a Scenario object. The route planner uses the startposition and the current orientation of the vehicle to develop a so called planning problem. Orientation can be calculated on the Odometry and the GNUU data from the sensoring area. The Output will be the so called route planner with the needed lanelet model.

Input: 
* Odometry data (sensoring)
* GNUU data (sensoring)

Output:
* lanelet model (local path planning, decision making)

---

## **Local Path Planning**

Local Planner updates the current route. If Decision Making detects an obstacle, the planner has to choose an alternative Lanelet to avoid the obstacle. Therefor the obstacle position on the lanelet plan will be detected. The vehicle chooses the next lanelet, where no obstacle collision occurs.

### <u>_Velocity profile_</u>

The Local Path Planer receives the lanelet points and the path to drive. The local planner creates a velocity profile on the calculated trajectorie. Curvature, crossings and traffic lights influence the profile. This will be calculated directly after the preplanning created a trajectorie. The current speed and the braking distance is held and calculated by the Acting side. The velocity value is published to the acting side. If the velocity is to high or to low, the Acting will realize and sends changes to adapt the speed of the vehicle.

Inpout:

* Trajectory points (preplanning)

Output:

* Max. Velocity (Acting)


### <u>_Update path_</u>

The update path module receives a command from the decision maker, that the actual path is not possible to drive. It uses the lanelet modell and chooses the next possible lanelet and updates the trajectory. It also tells the velocity profile to update the new trajectory.

Input:

* lanelet modell (preplanning)
* update command (decision making)
* information about blocked lanelets (decision making, perception)

Output:
* updated trajectory (acting, decision making)
* update command (velocity profile)

### <u>_Measure distance_</u>

This module measures the distance to obstacles, especially cars, with the Lidar Sensor. The current distance value is published to the acting side. Keeping a safe distance is calculated by the acting.

Input:

* Lidar Sensor data (perception, sensoring)

Output:

* distance value (acting)

---
## **Decision Making**

Obstacle detection is based on the sensor data from the perception area. If an obstacle is recognized, the decision making sends a message to the local path planning. The System chooses another lanelet.
With the Lanelets it is possible to give a prediciton for other objects and the vehicle itself, by following the lanelet direction of an object.

The decision making can be implemented with a state machine. For every incoming command (from the perception) must be a state defined. The system needs to make good predicitions to avoid collisions. The Perception data and the Lanelet modell are the main input for the system.

Input:

* Lanelet data (preplanning, local path planning)
* perception data (traffic lights situation, pedestrians,...)

Output:

* updated driving status (acting, local path planning)
* Lanelet data (acting)

Occupacy grid and Markov Modell


---
## Next steps

* Implement the commonroad route planner (old projects and Gitlab TUM)
* Analyze Lanelet plan and be familiar with it (Which information can we additionally receive from the plan?)
* Choose the Decision Maker (Evaluate Markov Modell in combination with occupacy grid)
* Read Lidar Sensor data and calculate distances
* Publish available and needed data (data available in this stage) 
---
### Sources

https://github.com/ll7/paf21-2/tree/main/paf_ros/paf_planning

https://gitlab.lrz.de/tum-cps/commonroad-route-planner