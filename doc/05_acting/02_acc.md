# ACC - Adaptive Cruise Control

**Summary:** This page contains information about the ACC component implemented in acting

---

## Author

Julian Graf

## Date

06.02.2023

<!-- TOC -->
* [ACC - Adaptive Cruise Control](#acc---adaptive-cruise-control)
  * [Author](#author)
  * [Date](#date)
  * [What is the ACC](#what-is-the-acc)
  * [When to use the ACC](#when-to-use-the-acc)
  * [How to turn on the ACC](#how-to-turn-on-the-acc)
  * [How to turn off the ACC](#how-to-turn-off-the-acc)
  * [How to test the ACC](#how-to-test-the-acc)
<!-- TOC -->

## What is the ACC

The [ACC](https://en.wikipedia.org/wiki/Adaptive_cruise_control) will publish the vehicle speed to maintain a safe distance to vehicle ahead.
An optimal distance to keep is therefore calculated using $dist = - \frac{1}{2} \cdot a \cdot (1s)^2 + v \cdot 1s + v \cdot 1s$.
This gives 1 second to react and 1s to break before hitting the car in front suddenly breaking.
The speed is then calculated using a [PID controller](https://en.wikipedia.org/wiki/PID_controller).

To do this, the node needs the current distance to the vehicle in front in meters via the topic `acc_distance`.
The ACC will automatically turn off, if this topic isn't published for more than one second.
The node also need to receive the current speed published by the carla bridge.

The ACC will publish he calculated speed as `max_velocity`.
The ACC might also trigger emergency breaking if a collisions seems likely.

## When to use the ACC

The ACC is best used when following a road behind another car.
It should only be used in more complicated situations with caution.
The ACC needs to know the distance to the car in front.
If this distance isn't published for more than one second, The ACC will automatically turn off.
The ACC only takes this distance in account.
This can for example lead to too high speed in tight corners or when driving around obstacles.

## How to turn on the ACC

The ACC tries to turn on as soon as a `acc_distance` is published.

The ACC can not turn on if  the published distance is too low, so the car in front is already too close.
The car in front has to have at least 70% of the optimal distance.

## How to turn off the ACC

The ACC can be turned off by publishing `acc_distance = -1`.

The ACC will also turn off automatically if the distance to the car in front gets to low.
If it is lower than half of the optimal distance, emergency breaking will be triggered.
If the `acc_distance` isn't published for more than one second, the ACC will turn off and publish a last target speed of 0.

## How to test the ACC

The ACC can be tested using the `AccDistancePublisherDummy`.
To do so, enable the `AccDistancePublisherDummy` in the acting launch file.
