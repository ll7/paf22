
# Based on https://github.com/ll7/paf21-1/blob/master/components/carla_ros_bridge/Dockerfile
FROM kstein/carla:leaderboard as carla

# supply the base image with an environment supporting ROS UI via x11
FROM osrf/ros:noetic-desktop-full-focal
COPY --from=carla /home/carla/PythonAPI /opt/carla/PythonAPI

ARG USERNAME=carla
ARG USER_UID=999
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME

# install rendering dependencies for rviz / rqt
RUN sudo apt-get update \
  && sudo apt-get install -y -qq --no-install-recommends \
    libxext6 libx11-6 libglvnd0 libgl1 \
    libglx0 libegl1 freeglut3-dev \
  && sudo rm -rf /var/lib/apt/lists/*


# override python path, carla pip package path didn't exist and was using Python 3.7 instead of 2.7
ENV PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3/dist-packages
ENV PYTHONPATH=$PYTHONPATH:/opt/carla/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg:/opt/carla/PythonAPI/carla

# install mlocate, pip, wget, git and some ROS dependencies for building the CARLA ROS bridge
RUN sudo apt-get update && sudo apt-get install -y \
        mlocate python3-pip wget git python-is-python3 \
        ros-noetic-ackermann-msgs \ros-noetic-derived-object-msgs \
        ros-noetic-carla-msgs ros-noetic-pcl-conversions \
        ros-noetic-rviz ros-noetic-rqt ros-noetic-pcl-ros && \
    sudo rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]
WORKDIR /opt/carla-ros-bridge
ARG ROS_BRIDGE_GITHUB_URL=https://github.com/carla-simulator/ros-bridge
RUN mkdir -p catkin_ws/src
RUN git clone -b leaderboard-2.0 --recurse-submodules --single-branch $ROS_BRIDGE_GITHUB_URL catkin_ws/src/ros-bridge
RUN cd catkin_ws/src/ros-bridge/carla_ackermann_control && \
    sed -i '24 i \  <build_depend condition="$ROS_VERSION == 1">carla_msgs</build_depend>' package.xml
RUN source /opt/ros/noetic/setup.bash && \
    cd catkin_ws && rosdep update && rosdep install --from-paths src --ignore-src -r && \
    catkin_make
    

WORKDIR /
ENV PYTHONPATH=$PYTHONPATH:/opt/carla-ros-bridge/install/lib/python3/dist-packages

# install simple_pid
RUN python -m pip install pip --upgrade \
    && python -m pip install simple_pid pygame transformations

# install the scenario runner from GitHub leaderboard-2.0 branch
ENV CARLA_ROOT=/opt/carla
ENV SCENARIO_RUNNER_ROOT=/opt/scenario_runner
RUN sudo mkdir $SCENARIO_RUNNER_ROOT && sudo chown $USERNAME:$USERNAME $SCENARIO_RUNNER_ROOT
RUN git clone -b leaderboard-2.0 --single-branch https://github.com/carla-simulator/scenario_runner.git $SCENARIO_RUNNER_ROOT
RUN echo 'pexpect' >> $SCENARIO_RUNNER_ROOT/requirements.txt && \
    python -m pip install -r $SCENARIO_RUNNER_ROOT/requirements.txt

ENV CARLA_SIM_HOST=localhost
ENV CARLA_SIM_WAIT_SECS=15
ENV SCENARIO_RUNNER_PATH=/opt/scenario_runner

WORKDIR /catkin_ws
RUN mkdir src
RUN source /opt/ros/noetic/setup.bash && catkin_init_workspace src
RUN sudo mkdir /workspace && sudo chown $USERNAME:$USERNAME /workspace

# Add agent code
COPY ./code /workspace/code

RUN ln -s /workspace/code/* /catkin_ws/src

RUN source /opt/ros/noetic/setup.bash && catkin_make

ADD ./build/docker/agent/entrypoint.sh /entrypoint.sh
ADD ./build/docker/agent/agent.launch /agent.launch

ENTRYPOINT ["/entrypoint.sh"]
CMD ["roslaunch", "/agent.launch"]