#!/usr/bin/env bash
# b5 Taskfile, see https://git.team23.de/build/b5 for details

# shortcuts for commands documented here
# http://wiki.ros.org/ROS/CommandLineTools#Common_user_tools

task:roscommand() {
 # seems necessary to source the setup file on every call
 docker:container_run agent /bin/bash -c "source /opt/ros/noetic/setup.bash && ${@}"
}

task:rosbag() {
 task:roscommand "rosbag ${@}"
}

task:ros_readbagfile() {
 task:roscommand "ros_readbagfile ${@}"
}

task:rosbash() {
 task:roscommand "rosbash ${@}"
}

task:roscd() {
 task:roscommand "roscd ${@}"
}

task:rosclean() {
 task:roscommand "rosclean ${@}"
}

task:roscore() {
 task:roscommand "roscore ${@}"
}

task:rosdep() {
 task:roscommand "rosdep ${@}"
}

task:rosed() {
 task:roscommand "rosed ${@}"
}

task:roscreate-pkg() {
 task:roscommand "roscreate-pkg ${@}"
}

task:roscreate-stack() {
 task:roscommand "roscreate-stack ${@}"
}

task:rosrun() {
 task:roscommand "rosrun ${@}"
}

task:roslaunch() {
 task:roscommand "roslaunch ${@}"
}

task:roslocate() {
 task:roscommand "roslocate ${@}"
}

task:rosmake() {
 task:roscommand "rosmake ${@}"
}

task:rosmsg() {
 task:roscommand "rosmsg ${@}"
}

task:rosnode() {
  additionalArguments="${@:1}"
 task:roscommand "rosnode ${@}"
}

task:rospack() {
 task:roscommand "rospack ${@}"
}

task:rosparam() {
 task:roscommand "rosparam ${@}"
}

task:rossrv() {
 task:roscommand "rossrv ${@}"
}

task:rosservice() {
 task:roscommand "rosservice ${@}"
}

task:rosstack() {
 task:roscommand "rosstack ${@}"
}

task:rostopic() {
 task:roscommand "rostopic ${@}"
}

task:rosversion() {
 task:roscommand "rosversion ${@}"
}
task:rqt_graph() {
  task:roscommand "rqt_graph ${@}"
}
