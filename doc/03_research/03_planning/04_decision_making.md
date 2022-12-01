# Decision-making module

**Summary:** This page gives a brief summary over possible decision-making choices their ad- and disadvantages as well as the opportunity to interchange them later on. Also, possible implementation options for those concepts are given.

---

## Author

Josef Kircher

## Date

01.12.2022

## Prerequisite

---
<!-- TOC -->
* [Decision-making module](#decision-making-module)
  * [Author](#author)
  * [Date](#date)
  * [Prerequisite](#prerequisite)
  * [Decision-making algorithms](#decision-making-algorithms)
    * [State machine](#state-machine)
      * [Advantages](#advantages)
      * [Disadvantages](#disadvantages)
    * [Markov Chain](#markov-chain)
      * [Advantages](#advantages)
      * [Disadvantages](#disadvantages)
    * [Behaviour Tree](#behaviour-tree)
      * [Advantages](#advantages)
      * [Disadvantages](#disadvantages)
  * [Python ROS libraries for these decision-making algorithms](#python-ros-libraries-for-these-decision-making-algorithms)
    * [Sources](#sources)
<!-- TOC -->

## Decision-making algorithms

### State machine

A finite-state machine (FSM) or finite-state automaton (FSA, plural: automata), finite automaton, or simply a state machine, is a mathematical model of computation.
It is an abstract machine that can be in exactly one of a finite number of states at any given time.
The FSM can change from one state to another in response to some inputs; the change from one state to another is called a transition.
An FSM is defined by a list of its states, its initial state, and the inputs that trigger each transition. Finite-state machines are of two types—deterministic finite-state machines and non-deterministic finite-state machines. A deterministic finite-state machine can be constructed equivalent to any non-deterministic one.

#### Advantages

* easy to implement
* we know most of the scenarios (finite state space)
* previous groups have solutions we could adapt/extend

#### Disadvantages

* many states necessary

### Markov Chain

A Markov chain or Markov process is a stochastic model describing a sequence of possible events in which the probability of each event depends only on the state attained in the previous event.
A countably infinite sequence, in which the chain moves state at discrete time steps, gives a discrete-time Markov chain. A continuous-time process is called a continuous-time Markov chain. It is named after the Russian mathematician Andrey Markov.

#### Advantages

* possible to build Markov Chain from State machine
* experience from previous projects
* only depends on current state ("memorylessness")

#### Disadvantages

* might be complicated to implement
* probabilities for transitions might need to be guessed, empirically estimated

### Decision Tree

A decision tree is a decision support tool that uses a tree-like model of decisions and their possible consequences, including chance event outcomes, resource costs, and utility.
It is one way to display an algorithm that only contains conditional control statements. Decision trees are commonly used in operations research, specifically in decision analysis, to help identify a strategy most likely to reach a goal, but are also a popular tool in machine learning.

#### Advantages

* easy implementation
* tree like structure usable in Machine Learning (Random Forest e.g.)

#### Disadvantages

* multiple decision trees necessary
* prediction independent of previous state

## Previous approaches

### PAF21-1

**State machine**

* 2 state machines: one for maneuvers, one for speed control
* Speed control more complex, when to brake seems like the most challenging task

**Take away**

* Some states seem to be comparable to what we are required to accomplish by the leaderboard
* Our task might be more complex, needs additional states and transitions
* I'm uncertain about an extra speed state, might be easier to handle that more locally by the local planner, maybe in combination with an observer element that keeps track of the surrounding by processing the information from `Perception`

### PAF21-2

**State machine**

* state machine integrated in local planner
* obstacle planner for dynamic obstacles (pedestrians, cars, bicycles)
* useful parameters which we could adapt
* path prediction for obstacles
* obstacles are only interesting if the cross the path of the ego vehicle

## Python ROS libraries for these decision-making algorithms

### Sources

<https://de.wikipedia.org/wiki/Markow-Kette#:~:text=Eine%20Markow%2DKette%20(englisch%20Markov,das%20Eintreten%20zuk%C3%BCnftiger%20Ereignisse%20anzugeben.>

<https://github.com/ll7/paf21-1/wiki/Decision-Making-Component>