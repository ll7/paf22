# Behaviour Tree - Behaviours Specification

**Disclaimer**: As we mainly built our decision tree on the previous [PAF project](https://github.com/ll7/psaf2), most part of the documentation was added here and adjusted to the changes we made.

**Conditions** never return `RUNNING` but only `FAILURE` and `SUCCESS`.

**Actions** should not return `FAILURE` (only special cases), but only `RUNNING` for ongoing and `SUCCESS` for finished.

## Road Features Subtree - Intersection

******
******

### _Condition:_ Intersection Ahead

Read the information from a topic from Perception.

**OnUpdate:** Return `SUCCESS` if intersection ahead, `FAILURE` otherwise.
******

### _Action:_ Approach Intersection

Slow down and stop at the traffic lights/hold line.

**OnUpdate:** If ongoing, return `RUNNING`, return `SUCCESS` after car has stopped.

**OnTerminate:** Cancel this action.

******

### _Action:_ Wait

Wait for the interscection to be free/traffic lights to become green.

**OnUpdate:** If not free to go, return `RUNNING`, otherwise `SUCCESS`.

**OnTerminate:** Cancel this action.

******

### _Action:_ Enter Intersection

Enter Intersection. If required, stop again (e.g. turn left).

**Initialize:** Set speed.

**OnUpdate:** Return `SUCCESS` if free to go without stopping again, return `RUNNING` if car still might have to stop (e.g. turn left).

**OnTerminate:** Cancel this action.

******

### _Action:_ Leave Intersection

Leave Intersection.

**OnUpdate:** If still in intersection, return `RUNNING`, otherwise `SUCCESS` (when back to other lanelet) and set speed to speed limit.

**OnTerminate:** Cancel this action.

## Lane Switch Subtree

******
******

### _Condition:_ Not slowed down by car in front

There should be a function/node (see ACC) that detects cars in front. It should write the information to a topic and this behaviour should query that topic. Additional information (e.g. car speed) should also be published in the topic (and written to the blackboard).

**OnUpdate:** Return `SUCCESS` if no slow car ahead, `FAILURE` otherwise.

## Lane Switch Subtree - Multilane

******
******

### _Condition_: Multi-Lane

**OnUpdate:** Return `SUCCESS` if multilane, `FAILURE` otherwise.

******

### _Condition_: Left-Lane available

**OnUpdate:** Return `SUCCESS` if left-lane available, `FAILURE` otherwise.

******

### _Action_: Wait for left lane free

Should have timeout.

**Initialize:** Set timeout.

**OnUpdate:**  If ongoing, return `RUNNING`, if lane free return `SUCCESS`, if timeout return `FAILURE`.

**OnTerminate:** Cancel this action.

******

### _Action_: Switch to lane left

**OnUpdate:**  If ongoing, return `RUNNING`, return `SUCCESS` after finishing.

**OnTerminate:** Cancel this action.

## Lane Switch Subtree - Single-Lane

******

******

### _Condition_: Single-line with dotted line

**OnUpdate:** Return `SUCCESS` if Single-line with dotted line, `FAILURE` otherwise.

******

### _Condition:_ Overtaking possible

Check if overtaking is possible (cars on other lane). In particular check cars ahead of the car to overtake.

**OnUpdate:** Return `SUCCESS` if possible, `FAILURE` otherwise.

******

### _Action_: Overtake

Accelerate, overtake car until there is space on the right lane.

**OnUpdate:**  If ongoing, return `RUNNING`, return `SUCCESS` if there is free space.

**OnTerminate:** Cancel this action.

******

### _Action_: Switch lane right

Go back to right lane.

**OnUpdate:**  If ongoing, return `RUNNING`, otherwise `FAILURE` or `SUCCESS`.

**OnTerminate:** Cancel this action.

## Lane Switch Subtree - Right-hand drive

******
******

### _Condition:_ Right lane available

**OnUpdate:** Return `SUCCESS` if right lane available, `FAILURE` otherwise.

******

### _Condition:_ Not slowed down by car in front right

There should be a function/node that detects cars in front (right). It should write the information to a topic and this behaviour should query that topic. Additional information (e.g. car speed) should also be published in the topic (and written to the blackboard).

**OnUpdate:** Return `SUCCESS` if no slow car ahead, `FAILURE` otherwise.

******

### _Action_: wait for lane right free

**OnUpdate:**  If ongoing, return `RUNNING`, otherwise `FAILURE` or `SUCCESS`.

**OnTerminate:** Cancel this action.

## Cruise

******
******

### _Action:_ Cruising

**OnUpdate:** Always return `RUNNING`. Set ACC.

**OnTerminate:** Cancel this action.
