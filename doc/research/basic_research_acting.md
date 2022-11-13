# Basic research acting

**Summary:** On this page you can find the results of the basic research on Acting.


---
### Author

Julian Graf

### Date

13.11.2022

---
[[TOC]] 

# Objective
- Safety
- driving comfort?
- 
# Solutions from old PAF projects

## Paf 20/1
- modified carla_ackermann_control
- input: twist-msgs (velocity)
- velocity control: PID
- lateral control: PD

Sources:
- https://github.com/ll7/psaf2/wiki/Path-Tracking-Algorithmen
- https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf

## Paf 21/1
- input: waypoints
- curve detection: returns distance to next curve
- calculation of max curve speed as sqrt(friction_coefficient  * gravity_accel * radius)
- in Curve: naive Controller
- on straights: stanley controller
- interface to rosbridge
Sources: 
- https://github.com/ll7/paf21-1/wiki/Vehicle-Controller
- https://dingyan89.medium.com/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081
- https://www.researchgate.net/publication/347286170_A_path-tracking_algorithm_using_predictive_Stanley_lateral_controller

## Paf 21/2 and Paf 20/2
- input: odometry(position and velocity with uncertainty), local path
- lateral: Stanley Controller
- speed controller: pid
- ACC: (speed, distance) -> PID
- Unstuck-Routine (drive backwards)
- Emergency Modus: fastest possible braking ([Tests](https://github.com/ll7/paf21-2/blob/main/docs/paf_actor/backwards/braking.md) -> handbrake with throttle, 30° steering and reverse)

Sources:
- https://github.com/ll7/paf21-2/tree/main/paf_ros/paf_actor#readme
- https://github.com/ll7/psaf2
- https://hal.archives-ouvertes.fr/hal-02459398/document

# Lateral control


# Velocity control
driving comfort?
ACC (adaptive cruise control) 


# Interface


# Limits

## weather

# Visualization 


# Additional functionality 
## Emergency brake assistant 
## Parking
## U-Turn
## Driving backwards
## Unstuck routine


# Potential next Issues


# Potential next Milestones


### Sources
