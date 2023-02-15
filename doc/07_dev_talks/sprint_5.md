# Dev talk - Sprint 5

**Summary:** For better communication a developer round is held in this sprint to improve communication and clarify responsibilities.

---

## Author

Josef Kircher
Julian Graf
Marco Riedenauer

## Date

01.02.2023

## Prerequisite

---
<!-- TOC -->
* [Dev talk - Sprint 5](#dev-talk---sprint-5)
  * [Author](#author)
  * [Date](#date)
  * [Prerequisite](#prerequisite)
  * [Planning](#planning)
    * [Suggestions](#suggestions)
    * [Things that need to handled](#things-that-need-to-handled)
    * [Results](#results)
  * [Acting](#acting)
  * [Perception](#perception)
    * [Sources](#sources)
<!-- TOC -->

## Planning

### Suggestions

How should be the naming convention for topics?

traffic light message:

    int state or string color
    float distance

traffic sign message:

    bool isStop
    float distance

speed limit message:

    float speedlimit
    float distance

acc message:
    bool activate/deactivate

### Things that need to handled

* Information from perception
  * Are there speed limits on the road? Can you handle these?
  * traffic signs are firstly limited to stop signs, I guess? Is the design modular enough to add different signs?
  * should there be a state as in the traffic light suggestion? Would keep the message short
  * is it alright to handle speed limits differently?
  * We might need to track crossing traffic for unsignalized intersections.

* Information from acting
  * is it simple enough to have a bool message?
  * this message would only be used to de-active in case of overturn and put back on afterwards and otherwise the ACC could turn on automatically when the distance to the car in front is below a threshold?

### Results

## Acting

no questions so far

## Perception

no questions so far

### Sources
