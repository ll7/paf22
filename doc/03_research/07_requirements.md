# Requirements

**Summary:** This page contains the requirements obtained from the Carla Leaderboard website as well as former projects in the `Praktikum Autonomes Fahren`

---

## Author

Josef Kircher, Simon Erlbacher

## Date

17.11.2022

## Prerequisite

---
<!-- TOC -->
* [Requirements](#requirements)
  * [Author](#author)
  * [Date](#date)
  * [Prerequisite](#prerequisite)
  * [Requirements from Leaderboard tasks](#requirements-from-leaderboard-tasks)
  * [Carla Leaderboard Score](#carla-leaderboard-score)
  * [Prioritized driving aspects](#prioritized-driving-aspects)
  * [more Content](#more-content)
    * [Sources](#sources)
<!-- TOC -->

## Requirements from Leaderboard tasks

* follow waypoints on a route
* don't deviate from route by more than 30 meters
* act in accordance with traffic rules
* don't get blocked
* complete 10 routes (2 weather conditions)

---

## Carla Leaderboard Score

The driving proficiency of an agent can be characterized by multiple metrics.

* `Driving score:` Product between route completion and infractions penalty
* `Route completion:` Percentage of the route distance completed by an agent
* `Infraction penalty:` The leaderboard tracks several types of infractions which reduce the score

Every agent starts with a base score of 1.0 at the beginning.

Types of infractions:

* collisions with pedestrians: 0.50
* collisions with other vehicles: 0.60
* collisions with static elements: 0.65
* running a red light: 0.70
* running a stop sign: 0.80
* scenario timeout (feature behaviours can block ego vehicle): 0.70
* Failure to maintain minimum speed: 0.70
* Failure to yield to emergency vehicle: 0.70
* Off-road driving: not considered towards the computation of the route completion score

---

## Prioritized driving aspects

There are different ways to prioritize the driving aspects mentioned in the document [08_use_cases](https://github.com/ll7/paf22/blob/482c1f5a201b52276d7b77cf402009bd99c93317/doc/03_research/08_use_cases.md).
The most important topics, in relation to this project, are the driving score and the safety aspect.
Also, it is appropriate to implement the basic features of an autonomous car first. The list is a mixture of the different approaches. Prioritizing from very important functionalities to less important features.

`Very important:`

* Recognize the street limitations
* Recognize pedestrians
* Follow the waypoints
* Recognize traffic lights
* Recognize obstacles
* Recognize cars in front of the agent (keep distance)
* Steering, accelerate, decelerate
* Street rules (no street signs available)
* Change lane (obstacles)

`Important:`

* Check Intersection
* Sense traffic (speed and trajectory)
* Predict traffic
* Emergency brake
* Sense length of ramp
* Recognize space (Turn into highway)
* Change lane (safe)
* Recognize emergency vehicle
* Recognize unexpected dynamic situations (opening door, bycicles,...)

`Less important:`

* Smooth driving (accelerate, decelerate, stop)
* Weather Condition
* Predict pedestrians

---

## more Content

### Sources

<https://www.markdownguide.org/cheat-sheet/>
