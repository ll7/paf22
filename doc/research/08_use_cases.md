# Use cases in Carla Leaderboard

**Summary:** This page contains a set of possible use cases containing a description of the scenario and the functions the agent has to have to pass that scenario.

---

## Author

Josef Kircher

## Date

21.11.2022

## Prerequisite

---
<!-- TOC -->
* [Title of wiki page](#title-of-wiki-page)
  * [Author](#author)
  * [Date](#date)
  * [Prerequisite](#prerequisite)
  * [Some Content](#some-content)
  * [more Content](#more-content)
    * [Sources](#sources)
<!-- TOC -->

## Control loss due to bad road condition

### Description

The ego-vehicle loses control due to bad conditions on the road and it must recover, coming back to its original lane.

### Pre-condition(Event)

loss of control

### Driving functions

* Control steering angle, throttle and brake to counter unexpected movements

* (Opt): Sense wheel friction to predict unexpected behaviour

### Outcome

Stabilize the car and continue driving

### Associated use cases


## Unprotected left turn at intersection

### Description

On an intersection with no specified left-turn arrow, the vehicle has to perform the following steps:

### Basic flow

1. Check for traffic lights
2. If traffic lights are off, look for street signs
3. If there are no street signs, the first to enter the intersection has priority
4. If traffic rules permit to enter the intersection, check if intersection is clear
5. Yield to oncoming traffic and bicycles
6. Check for pedestrians trying to cross the street
7. Perform the turn

### Pre-condition(Event)

Global route wants you to perform a left turn at an intersection

### Driving functions

* Sense street signs and traffic lights
* Observe the intersection
* Sense oncoming traffic
* Check indicator of oncoming traffic
* Sense pedestrians in your drive path
* Steer the vehicle in a left turn
* Predict if a turn is possible before oncoming traffic reaches the intersection

### Outcome

Turn left at the intersection without violating traffic rules

### Associated use cases
TODO

## Right turn at the intersection

### Description

On an intersection with no specified right-turn arrow, the vehicle has to perform the following steps:

### Basic flow

1. Check for traffic lights
2. If traffic lights are off, look for street signs
3. If there are no street signs, the first to enter the intersection has priority
4. If traffic rules permit to turn right, check if there is crossing traffic or pedestrians
5. Perform the turn

### Pre-condition(Event)

Global route wants you to perform a right turn at an intersection

### Driving functions

* Sense street signs and traffic lights
* Observe the intersection
* Sense crossing traffic
* Check indicator of crossing traffic
* Sense pedestrians in your drive path
* Steer the vehicle in a right turn
* Predict if a turn is possible before crossing traffic reaches the intersection

### Outcome

Turn right at the intersection without violating traffic rules

### Associated use cases

TODO

## Crossing negotiation at unsignalized intersection

### Description

On an intersection with no street signs or traffic lights, perform the following steps:

# Basic flow

1. Check for traffic lights
2. If traffic lights are off, look for street signs
3. If there are no street signs, the first to enter the intersection has priority
4. Check if intersection is clear
5. If not, wait until intersection is clear
6. Enter the intersection
7. Follow global path

### Pre-condition(Event)

No traffic lights or street signs are sensed

### Driving functions

* Sense street signs and traffic lights
* Observe the intersection
* Sense pedestrians in your drive path
* Steering the vehicle

### Outcome

Cross the intersection without violating traffic rules

### Associated use cases

TODO

## Crossing traffic running a red light at intersection

### Description

Avoid collision with crossing traffic running a red light

### Pre-condition(Event)

Vehicle enters intersection while having a red light

### Driving functions

* Sense street signs and traffic lights
* Observe the intersection
* Sense crossing traffic
* Emergency brake

### Outcome

Emergency brake to avoid collision

### Associated use cases

TODO

## Highway merge from on-ramp

### Description

Join the traffic on the highway from a ramp

### Basic flow

1. Check speed of vehicles on the next lane
2. Determine length of ramp
3. Adjust speed to find a gap in the traffic
4. Join the highway before ramp ends

### Pre-condition(Event)

Vehicle enters a highway

### Driving functions

* Sense speed of surrounding traffic
* Sense length of ramp
* Adjust speed to enter highway
* Turn into highway

### Outcome

Join the highway traffic without any traffic violation

### Associated use cases

TODO

### Sources

<https://leaderboard.carla.org/scenarios/>
