# Acting

**Summary:** This package contains all functions implemented for the acting component.

---

## Authors

Julian Graf

Gabriel Schwald

## Date

29.03.2023

---

<!-- TOC -->
* [Acting](#acting)
  * [Authors](#authors)
  * [Date](#date)
  * [Basics](#basics)
  * [Content of this package](#content-of-this-package)
    * [Longitudinal controllers](#longitudinal-controllers)
    * [Lateral controllers](#lateral-controllers)
    * [Vehicle controller](#vehicle-controller)
    * [Visualization](#visualization)
  * [more Content](#more-content)
<!-- TOC -->

## Basics

In order to further understand the general idea of our approach to the acting component please refer to  the documentation of our [research](../../doc/03_research/01_acting) and our planned [general definition](../../doc/01_general/04_architecture.md#acting).

---

## Content of this package

This package contain all acting nodes. For general information on what the acting does, see the [planned architecture](./../../doc/01_general/04_architecture.md#acting).

### Longitudinal controllers

The longitudinal controller is implemented as a PID velocity controller ([VelocityController](./src/acting/velocity_controller.py)). The acting also publishes a max velocity based onn upcoming curves ([ActingVelocityPublisher](./src/acting/acting_velocity_publisher.py)).
Also there is a [ACC](../../doc/05_acting/02_acc.md) implemented in acting ([Acc](./src/acting/acc.py)).

### Lateral controllers

There are two steering controllers implemented in the acting:

* Pure Persuit Controller
* Stanley Controller

For further information see [lateral controllers](./../../doc/05_acting/03_lateral_controller.md).

### Vehicle controller

The [VehicleController](./src/acting/vehicle_controller.py) collects all necessary msgs and publishes the [CarlaEgoVehicleControl](https://carla.readthedocs.io/en/0.9.8/ros_msgs/#carlaegovehiclecontrol) for the [Carla ros bridge](https://github.com/carla-simulator/ros-bridge).

### Visualization

For information about vizualizing the upcomming path in rviz see [Main frame publisher](../../doc/05_acting/04_main_frame_publisher.md)

## more Content

For more detailed information on all relevant components of the acting domain please refer to [the acting documentation](../../doc/05_acting).
