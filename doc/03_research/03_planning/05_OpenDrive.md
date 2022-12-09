# OpenDrive Format

**Summary:** Evaluate the reading of the OpenDrive map in other projects and outline recommended further steps.

---

## Authors

Simon Erlbacher

### Date

04.12.2022

---

<!-- TOC -->
* [OpenDrive Format](#opendrive-format)
  * [Authors](#authors)
    * [Date](#date)
  * [General](#general)
  * [Different Projects](#different-projects)
    * [PSAF1](#psaf1)
    * [PSAF2](#psaf2)
    * [paf21-2](#paf21-2)
    * [paf21-1](#paf21-1)
    * [Result](#result)
  * [More information about OpenDrive](#more-information-about-opendrive)
  * [Sources](#sources)
  
<!-- TOC -->

## General

The OpenDrive format provides a common base for describing road networks with extensible markup language (XML) syntax,
using the file extension xodr. The data that is stored in an ASAM OpenDRIVE file describes the geometry of roads, lanes
and objects, such as roadmarks on the road, as well as features along the roads, like signals
(traffic lights, stop signs,...). It is based on real and synthetic data.

## Different Projects

It is examined how the OpenDrive file is converted and read in other groups and projects.

### PSAF1

* Subscribed the OpenDrive information from the Carla Simulator
* Used the Commonroad Route Planner from TUM (in the project they used the now deprecated verison)
* This Route Planner converts the xdor file from the CarlaWorldInfo message automatically
* As a result they used a Lanelet model, which they enriched with additional information about
traffic lights and traffic signs
* This additional information comes from the Carla Simulator API

Result: We can't use this information from [psaf1]("https://github.com/ll7/psaf1/tree/master/psaf_ros/psaf_global_planner")
, because it is not allowed to use privileged information from the Carla Simulator

### PSAF2

* Same approach as described in PSAF1 above
* Same problem in [psaf2](https://github.com/ll7/psaf2/tree/main/Planning/global_planner) with this approach as
  mentioned in PSAF1

### paf21-2

* Same approach as described in PSAF1 above
* Same problem in [paf21-2](https://github.com/ll7/paf21-2#global-planner) with this approach as mentioned in PSAF1

### paf21-1

* Worked directly with the OpenDrive format
* There is a lot of information available
* They extracted some information from the xdor file to plan their trajectory
* They don't recommend to use this approach, because a lot of "black magic" is happening in their code

Result: The only possible way to get all the road information without using the Carla Simulator API

### Result

The Commonroad Route Planner can be used to convert the xdor file and generate a Lanelet Modell from it. But from
experiences with this tool, it is not recommended to use it. The planer loses a alot of information about the map
during the planning process. It would be better to convert and analyse the xdor file directly.

## More information about OpenDrive

* We can read the xdor file with the [ElementTree XML API](https://docs.python.org/3/library/xml.etree.elementtree.html)
* We can refactor the scripts from paf21-1 but as they described, it is a lot of code and hard to get a good
overview about it
* Also we have a different scenario, because we do not need to read the whole xdor file in the beginning. We need
to search for the relevant area
* The OpenDrive format contains a lot of information to extract
  * Every road section has a unique id
  * Road has a predecessor and a successor with its specific type (road, junction,...)
  * Information about signals and their position
  * Information about the reference lines (line which seperates lanes) and their layout (linear, arc, cubic curves)
  * Information about the maximum speed
  
![OpenDrive stop sign](../../00_assets/Stop_sign_OpenDrive.png)
Impression of the format
  
There are a lot of infomrations in the file. Also a lot of information, which is not relevant for our project.
For the next steps i recommend to read the file and in the first steps to find the road id where the agent is located
and the following roads required to reach the goal position.
Then we need to create the trajectory points from the agent to the goal position.
After that, we can add some more information about the signals to our trajectory.

## Start of the implementation

structure of the xodr files from the Simulator:

* header
* road (attributes: junction id (-1 if no junction), length, road id, Road name)
  * lanes
  * link (predecessor and successor with id nad contactpoints)
  * signals
  * type (contains max speed)
  * planView (contains information about the geometry and the line type (= reference line))
* controller (information about the controlled signals)
* junction

planView:

* x and y world coordinates
* hdg value for the orientation
* length value for the length of this road section

lane:

* lane id

## Sources

<https://carla.readthedocs.io/en/latest/core_map/>

<https://docs.python.org/3/library/xml.etree.elementtree.html>

<https://www.asam.net/index.php?eID=dumpFile&t=f&f=4422&token=e590561f3c39aa2260e5442e29e93f6693d1cccd#top-016f925e-bfe2-481d-b603-da4524d7491f>
