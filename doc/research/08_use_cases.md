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

## Highway cut-in from on-ramp

### Description

A vehicle tries to join the traffic on the highway from a ramp, the agent has to brake, switch lane or decelerate. 

### Basic flow

1. Sense vehicle on the on ramp
2. Check speed of vehicles on the left lane
3. If possible, change lane
4. Check speed of vehicle on on-ramp
5. Decelerate or brake to allow vehicle to merge into traffic

### Pre-condition(Event)

Vehicle enters a highway

### Driving functions

* Sense speed of surrounding traffic
* Adjust speed to let vehicle enter highway
* Change lane
* Decelerate
* Brake

### Outcome

Let vehicle join the highway traffic without any traffic violation

### Associated use cases

TODO

## static cut-in

### Description

A vehicle tries to join the lane of the agent, the agent has to brake, switch lane or decelerate. 

### Basic flow

1. Sense vehicle on the right lane and its intent to switch lanes
2. Check speed of vehicles on the left lane
3. If possible, change lane
4. Check speed of vehicle on the right
5. Decelerate or brake to allow vehicle to merge into our lane

### Pre-condition(Event)

Vehicle tries to cut-in

### Driving functions

* Sense speed of surrounding traffic
* Adjust speed to let vehicle enter lane
* Change lane
* Decelerate
* Brake

### Outcome

Let vehicle join the lane without any traffic violation

### Associated use cases

TODO

## Highway exit

### Description

Agent must cut through a lane of moving traffic to get to the off-ramp.

### Basic flow

1. Sense vehicle on the right lane
2. Check speed of vehicles on the right lane
3. Check distance to off-ramp
4. If possible, change lane
5. Accelerate or decelerate to change into right lane
6. Change lane
7. Change lane to off-ramp
8. Decelerate or brake when leaving high-way

### Pre-condition(Event)

Vehicle leaves a highway

### Driving functions

* Sense speed of surrounding traffic
* Sense distance to off-ramp
* Adjust speed to change lane
* Change lane
* Decelerate
* Brake

### Outcome

Vehicle exits the highway traffic without any traffic violation

### Associated use cases

TODO

## Yield to emergency vehicle

### Description

If an emergency vehicle arrives from behind on your lane, maneuver out of the way and let it pass.

### Basic flow

1. Sense emergency vehicle behind the agent
2. Check speed of vehicles on the surrounding lanes
3. If possible, change lane
4. Adjust speed to perform a lane change
5. Change lane
6. Let emergency vehicle pass

### Pre-condition(Event)

Emergency vehicle behind us

### Driving functions

* Sense emergency vehicle
* Sense speed of surrounding traffic
* Adjust speed to change lane
* Change lane

### Outcome

Let emergency vehicle pass without any traffic violation

### Associated use cases

TODO

## Obstacle in lane

### Description

The ego-vehicle encounters an obstacle blocking the lane and must perform a lane change to avoid it. The obstacle may be a construction site, an accident or a parked vehicle.

### Basic flow

1. Sense obstacle in the lane
2. Check speed of vehicles on the surrounding lanes
3. If possible, change lane
4. Decelerate or brake to avoid collision with obstacle
5. If a sufficient gap in the traffic opens, change lane
6. After obstacle remerge former lane

### Pre-condition(Event)

Obstacle on lane

### Driving functions

* Sense obstacles
* Sense speed of surrounding traffic
* Change lane
* Decelerate
* Brake

### Outcome

Pass an obstacle without any traffic violation

### Associated use cases

TODO

## Door Obstacle

### Description

The ego-vehicle encounters a parked vehicle opening a door into its lane and must maneuver to avoid it.

### Basic flow

1. Sense door obstacle in the lane
2. Check speed of vehicles on the surrounding lanes
3. If possible, change lane
4. Decelerate or brake to avoid collision with obstacle
5. If a sufficient gap in the traffic opens, change lane
6. After obstacle remerge former lane

### Pre-condition(Event)

Door opens in lane

### Driving functions

* Sense opening door
* Sense speed of surrounding traffic
* Change lane
* Decelerate
* Brake

### Outcome

Pass the open door without any traffic violation

### Associated use cases

TODO

## Slow moving hazard

### Description

The ego-vehicle encounters a slow moving hazard blocking part of the lane. The ego-vehicle must brake or maneuver to avoid it

### Basic flow

1. Sense slow moving hazard in the lane
2. Check speed of vehicles on the surrounding lanes
3. If possible, change lane
4. Decelerate or brake to avoid collision with obstacle
5. If a sufficient gap in the traffic opens, change lane
6. After obstacle remerge former lane

### Pre-condition(Event)

slow moving hazard(bicycle) in lane

### Driving functions

* Sense slow moving hazards
* Sense speed of surrounding traffic
* Change lane
* Decelerate
* Brake

### Outcome

Pass the slow moving hazard without any traffic violation

### Associated use cases

TODO

## Vehicle invading lane on bend

### Description

The ego-vehicle encounters an oncoming vehicles invading its lane on a bend due to an obstacle. It must brake or maneuver to the side of the road to navigate past the oncoming traffic.

### Basic flow

1. Sense vehicle on our lane
2. Decelerate or brake to avoid collision with vehicle
3. Steer to the right part of the lane
4. After vehicle passed continue on lane

### Pre-condition(Event)

Bend in the road and a vehicle invading our lane

### Driving functions

* Sense vehicle on our lane
* Decelerate
* Brake
* Move to right part of lane

### Outcome

Let vehicle pass without any traffic violation

### Associated use cases

TODO

### Sources

<https://leaderboard.carla.org/scenarios/>
