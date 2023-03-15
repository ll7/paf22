# Behaviour Tree - Behaviours Specification

**Conditions** never return _Running_ but only _Failure_ and _Success_.
**Actions** should not return _Failure_ (only special cases), but only _Running_ for ongoing and _Success_ for finished.

## Collision Avoidance Subtree

******
******

### _Condition:_ No Obstacle Ahead

There should be a function/node that detects obstacles. It should write the information to a topic and this behaviour should query that topic. Additional information (e.g. distance to obstacle) should also be published in the topic (and written to the blackboard).

**OnUpdate:** Return _Success_ if no obstacle ahead, _Failure_ otherwise.

******

### _Action:_ Replan Around Obstacle

Trigger function for local replaning, set timeout with respect to information published from obstacle detection.

**OnUpdate:** If timeout but planning not done or error occured, return _Failure_. Before timeout while still planning, return _Running_. If planning finishes in time, return _Success_ and publish new local plan.

**OnTerminate:** When this behaviour is termianted by higher node, cancel the planning (or at least do not publish the result).

## Road Features Subtree

******

The node that percepts the road features should take care that not more than one (and only the closest) feature should be published. This could be done via the global plan (offline planning of a sequence of road features).

## Road Features Subtree - Intersection

******
******

### _Condition:_ Intersection Ahead

Read the information from a topic from Perception.

**OnUpdate:** Return _Success_ if intersection ahead, _Failure_ otherwise.
******

### _Action:_ Approach Intersection

Slow down and stop at the traffic lights/marker. Choose and move to the required lane.
Activate turning signal.

**OnUpdate:** If ongoing, return _Running_, return _Success_ after car has stopped.

**OnTerminate:** Cancel this action.

******

### _Action:_ Wait

Wait for the interscection to be free/traffic lights to become green.

**OnUpdate:** If not free to go, return _Running_, otherwise _Success_.

**OnTerminate:** Cancel this action.

******

### _Action:_ Enter Intersection

Enter Intersection. If required, stop again (e.g. turn left).

**Initialize:** Set speed.

**OnUpdate:** Return _Success_ if free to go without stopping again, return _Running_ if car still might have to stop (e.g. turn left).

**OnTerminate:** Cancel this action.

******

### _Action:_ Leave Intersection

Leave Intersection.

**OnUpdate:** If still in intersection, return _Running_, otherwise _Success_ (when back to other lanelet) and set speed to speed limit.

**OnTerminate:** Cancel this action.

## Road Features Subtree - Roundabout

******
******

### _Condition:_ Roundabout Ahead

Read the information from a topic from Perception.

**OnUpdate:** Return _Success_ if roundabout ahead, _Failure_ otherwise.
******

### _Action:_ Approach Roundabout

Slow down and stop at the roundabout entrance. Choose lane.

**OnUpdate:** If ongoing, return _Running_, return _Success_ after stopping.

**OnTerminate:** Cancel this action.

******

### _Action:_ Wait

Wait for the lane in Roundabout to be free.

**OnUpdate:** If not free to go, return _Running_, otherwise _Success_.

**OnTerminate:** Cancel this action.

******

### _Action:_ Enter

Enter Roundabout.

**Initialize:** Set speed.

**OnUpdate:** If ongoing, return _Running_, otherwise _Success_.

**OnTerminate:** Cancel this action.

******

### _Action:_ Leave

Leave Roundabout. Check for lane switch that needs to be done.

**OnUpdate:** If still in Roundabout, return _Running_, otherwise _Success_ (when back to other lanelet) and set speed to speed limit.

**OnTerminate:** Cancel this action.

## Road Features Subtree - Stop

******
******

### _Condition:_ Stop Ahead

Read the information from a topic from Perception. (Stop means traffic ligths, zebra crossing, ...)

**OnUpdate:** Return _Success_ if intersection ahead, _Failure_ otherwise.
******

### _Action:_ Approach

Slow down and stop at the marker.

**OnUpdate:** If ongoing, return _Running_, otherwise _Success_.

**OnTerminate:** Cancel this action.

******

### _Action:_ Wait

Wait for go.

**OnUpdate:** If ongoing, return _Running_, if free return _Success_ and set speed.

**OnTerminate:** Cancel this action.

******

### _Action:_ Go

Continue and pass obstacle.

**OnUpdate:** If ongoing, return _Running_, otherwise _Success_ (after car has passed stop).

**OnTerminate:** Cancel this action.

## Lane Switch Subtree

******
******

### _Condition:_ Not slowed down by car in front

There should be a function/node (see ACC) that detects cars in front. It should write the information to a topic and this behaviour should query that topic. Additional information (e.g. car speed) should also be published in the topic (and written to the blackboard).

**OnUpdate:** Return _Success_ if no slow car ahead, _Failure_ otherwise.

## Lane Switch Subtree - Multilane

******
******

### _Condition_: Multi-Lane

**OnUpdate:** Return _Success_ if multilane, _Failure_ otherwise.

******

### _Condition_: Left-Lane available

**OnUpdate:** Return _Success_ if left-lane available, _Failure_ otherwise.

******

### _Action_: Wait for left lane free

Should have timeout.

**Initialize:** Set timeout.

**OnUpdate:**  If ongoing, return _Running_, if lane free return _Success_, if timeout return _Failure_.

**OnTerminate:** Cancel this action.

******

### _Action_: Switch to lane left

**OnUpdate:**  If ongoing, return _Running_, return _Success_ after finishing.

**OnTerminate:** Cancel this action.

## Lane Switch Subtree - Single-Lane

******

******

### _Condition_: Single-line with dotted line

**OnUpdate:** Return _Success_ if Single-line with dotted line, _Failure_ otherwise.

******

### _Condition:_ Overtaking possible

Check if overtaking is possible (cars on other lane). In particular check cars ahead of the car to overtake.

**OnUpdate:** Return _Success_ if possible, _Failure_ otherwise.

******

### _Action_: Overtake

Accelerate, overtake car until there is space on the right lane.

**OnUpdate:**  If ongoing, return _Running_, return _Success_ if there is free space.

**OnTerminate:** Cancel this action.

******

### _Action_: Switch lane right

Go back to right lane.

**OnUpdate:**  If ongoing, return _Running_, otherwise _Failure_ or _Success_.

**OnTerminate:** Cancel this action.

## Lane Switch Subtree - Right-hand drive

******
******

### _Condition:_ Right lane available

**OnUpdate:** Return _Success_ if right lane available, _Failure_ otherwise.

******

### _Condition:_ Not slowed down by car in front right

There should be a function/node that detects cars in front (right). It should write the information to a topic and this behaviour should query that topic. Additional information (e.g. car speed) should also be published in the topic (and written to the blackboard).

**OnUpdate:** Return _Success_ if no slow car ahead, _Failure_ otherwise.

******

### _Action_: wait for lane right free

**OnUpdate:**  If ongoing, return _Running_, otherwise _Failure_ or _Success_.

**OnTerminate:** Cancel this action.

## Cruise

******
******

### _Action:_ Cruising

**OnUpdate:** Always return _Running_. Set ACC.

**OnTerminate:** Cancel this action.
