#!/usr/bin/env bash
# b5 Taskfile, see https://git.team23.de/build/b5 for details

task:rosbag() {
  additionalArguments="${@:1}"
  task:shell agent "rosbag" ${additionalArguments:-}
}

task:ros_readbagfile() {
  additionalArguments="${@:1}"
  task:shell agent "ros_readbagfile" ${additionalArguments:-}
}

task:rosbash() {
  additionalArguments="${@:1}"
  task:shell agent "rosbash" ${additionalArguments:-}
}

task:roscd() {
  additionalArguments="${@:1}"
  task:shell agent "roscd" ${additionalArguments:-}
}

task:rosclean() {
  additionalArguments="${@:1}"
  task:shell agent "rosclean" ${additionalArguments:-}
}

task:roscore() {
  additionalArguments="${@:1}"
  task:shell agent "roscore" ${additionalArguments:-}
}

task:rosdep() {
  additionalArguments="${@:1}"
  task:shell agent "rosdep" ${additionalArguments:-}
}

task:rosed() {
  additionalArguments="${@:1}"
  task:shell agent "rosed" ${additionalArguments:-}
}

task:roscreate-pkg() {
  additionalArguments="${@:1}"
  task:shell agent "roscreate-pkg" ${additionalArguments:-}
}

task:roscreate-stack() {
  additionalArguments="${@:1}"
  task:shell agent "roscreate-stack" ${additionalArguments:-}
}

task:rosrun() {
  additionalArguments="${@:1}"
  task:shell agent "rosrun" ${additionalArguments:-}
}

task:roslaunch() {
  additionalArguments="${@:1}"
  task:shell agent "roslaunch" ${additionalArguments:-}
}

task:roslocate() {
  additionalArguments="${@:1}"
  task:shell agent "roslocate" ${additionalArguments:-}
}

task:rosmake() {
  additionalArguments="${@:1}"
  task:shell agent "rosmake" ${additionalArguments:-}
}

task:rosmsg() {
  additionalArguments="${@:1}"
  task:shell agent "rosmsg" ${additionalArguments:-}
}

task:rosnode() {
  additionalArguments="${@:1}"
  task:shell agent "rosnode" ${additionalArguments:-}
}

task:rospack() {
  additionalArguments="${@:1}"
  task:shell agent "rospack" ${additionalArguments:-}
}

task:rosparam() {
  additionalArguments="${@:1}"
  task:shell agent "rosparam" ${additionalArguments:-}
}

task:rossrv() {
  additionalArguments="${@:1}"
  task:shell agent "rossrv" ${additionalArguments:-}
}

task:rosservice() {
  additionalArguments="${@:1}"
  task:shell agent "rosservice" ${additionalArguments:-}
}

task:rosstack() {
  additionalArguments="${@:1}"
  task:shell agent "rosstack" ${additionalArguments:-}
}

task:rostopic() {
  additionalArguments="${@:1}"
  task:shell agent "rostopic" ${additionalArguments:-}
}

task:rosversion() {
  additionalArguments="${@:1}"
  task:shell agent "rosversion" ${additionalArguments:-}
}
