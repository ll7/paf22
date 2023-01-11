# SMACH

**Summary:** SMACH is a python library used to generate and inspect state machines. It has a very clear structure and is easy to understand, so it is used in this project.

---

## Author

Josef Kircher

## Date

09.01.2023

## Prerequisite

---
<!-- TOC -->
* [SMACH](#smach)
  * [Author](#author)
  * [Date](#date)
  * [Prerequisite](#prerequisite)
  * [Getting started](#getting-started)
  * [What is SMACH?](#what-is-smach)
  * [Examples](#examples)
    * [Sources](#sources)
<!-- TOC -->
## Getting started

SMACH is part of this project's dockerfile, so no setup is required.

## What is SMACH?

SMACH is a task-level architecture for rapidly creating complex robot behavior. At its core, SMACH is a ROS-independent Python library to build hierarchical state machines.
SMACH is a new library that takes advantage of very old concepts in order to quickly create robust robot behavior with maintainable and modular code.

To get a better understanding of what SMACH is and what it does please visit [Smach ROS Wiki](http://wiki.ros.org/smach)

## Examples

There are many examples for the different functions of SMACH.

They are all listed [here](http://wiki.ros.org/smach/Tutorials).

To get a basic understanding for what we need in this project the following tutorial is recommended.

* [Simple State Machine](http://wiki.ros.org/smach/Tutorials/Simple%20State%20Machine)

To run this example execute the following steps.

1. call `b5 update` to update docker container
2. call `b5 run` to start container
3. in a second shell call `b5 shell`
4. run `./planning/executive_smach_examples/state_machine_simple.py` to execute the example

### Sources

<http://wiki.ros.org/smach>
