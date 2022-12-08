
# Based on https://github.com/ll7/paf21-1/blob/master/components/carla_ros_bridge/Dockerfile
FROM kstein/carla:leaderboard as carla

# supply the base image with an environment supporting ROS UI via x11
FROM osrf/ros:noetic-desktop-full-focal
COPY --from=carla /home/carla/PythonAPI /opt/carla/PythonAPI

ARG USERNAME=carla
ARG USER_UID=999
ARG USER_GID=$USER_UID


# install rendering dependencies for rviz / rqt
RUN apt-get update \
  && apt-get install -y -qq --no-install-recommends \
    libxext6 libx11-6 libglvnd0 libgl1 \
    libglx0 libegl1 freeglut3-dev

# override python path, carla pip package path didn't exist and was using Python 3.7 instead of 2.7
ENV PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages
ENV PYTHONPATH=$PYTHONPATH:/opt/carla/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg:/opt/carla/PythonAPI/carla

# install mlocate, pip, wget, git and some ROS dependencies for building the CARLA ROS bridge
RUN apt-get update && apt-get install -y \
        mlocate python3-pip wget git python-is-python3 \
        ros-noetic-ackermann-msgs ros-noetic-derived-object-msgs \
        ros-noetic-carla-msgs ros-noetic-pcl-conversions \
        ros-noetic-rviz ros-noetic-rqt ros-noetic-pcl-ros ros-noetic-rosbridge-suite ros-noetic-rosbridge-server

SHELL ["/bin/bash", "-c"]

# Create and apply the non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
USER $USERNAME

# create a catkin workspace
WORKDIR /catkin_ws
# make sure the catkin workspace belongs to the user
RUN sudo chown $USERNAME:$USERNAME /catkin_ws && mkdir src

# install the CARLA ROS bridge
ARG ROS_BRIDGE_GITHUB_URL=https://github.com/carla-simulator/ros-bridge
RUN git clone -b leaderboard-2.0 --recurse-submodules --single-branch $ROS_BRIDGE_GITHUB_URL src/ros-bridge
# apply patch to fix the build of carla_ros_bridge
RUN cd src/ros-bridge/carla_ackermann_control && \
    sed -i '24 i \  <build_depend condition="$ROS_VERSION == 1">carla_msgs</build_depend>' package.xml
RUN source /opt/ros/noetic/setup.bash && \
    rosdep update && rosdep install --from-paths src --ignore-src -r && \
    catkin_make install

ENV CARLA_ROS_BRIDGE_ROOT=/catkin_ws/src/ros-bridge

# avoid python warnings about missing bin directory in PATH
# (as we're not running as root, pip installs into ~/.local/bin)
ENV PATH=$PATH:/home/$USERNAME/.local/bin

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

# install the leaderboard from GitHub leaderboard-2.0 branch
ENV LEADERBOARD_ROOT=/opt/leaderboard
RUN sudo mkdir $LEADERBOARD_ROOT && sudo chown $USERNAME:$USERNAME $LEADERBOARD_ROOT
RUN git clone -b leaderboard-2.0 --single-branch  https://github.com/carla-simulator/leaderboard.git $LEADERBOARD_ROOT
RUN python -m pip install -r $LEADERBOARD_ROOT/requirements.txt
RUN sudo mkdir /opt/leaderboard-py3 && sudo chown $USERNAME:$USERNAME /opt/leaderboard-py3 && \
    ln -s $LEADERBOARD_ROOT/leaderboard /opt/leaderboard-py3/leaderboard && \
    ln -s $SCENARIO_RUNNER_ROOT/srunner /opt/leaderboard-py3/srunner

RUN python -m pip install roslibpy


# environment variables for the leaderboard_evaluator
ENV ROUTES=$LEADERBOARD_ROOT/data/routes_devtest.xml
ENV REPETITIONS=1
ENV DEBUG_CHALLENGE=1
#ENV TEAM_AGENT=$LEADERBOARD_ROOT/leaderboard/autoagents/human_agent.py
ENV TEAM_AGENT=/workspace/code/agent/src/agent/agent.py
ENV CHECKPOINT_ENDPOINT=/workspace/code/simulation_results.json
ENV CHALLENGE_TRACK=MAP

ENV CARLA_SIM_HOST=localhost
ENV CARLA_SIM_WAIT_SECS=15
ENV SCENARIO_RUNNER_PATH=/opt/scenario_runner

# setup python path for PyCharm integration
RUN echo /catkin_ws/install/lib/python3/dist-packages >> /home/$USERNAME/.local/lib/python3.8/site-packages/carla.pth && \
    echo /catkin_ws/devel/lib/python3/dist-packages >> /home/$USERNAME/.local/lib/python3.8/site-packages/carla.pth && \
    echo /opt/ros/noetic/lib/python3/dist-packages >> /home/$USERNAME/.local/lib/python3.8/site-packages/carla.pth && \
    echo /opt/ros/noetic/lib/python3/dist-packages >> /home/$USERNAME/.local/lib/python3.8/site-packages/carla.pth && \
    echo /opt/leaderboard-py3 >> /home/$USERNAME/.local/lib/python3.8/site-packages/carla.pth && \
    echo /opt/carla/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg >> /home/$USERNAME/.local/lib/python3.8/site-packages/carla.pth

RUN sudo mkdir /workspace && sudo chown $USERNAME:$USERNAME /workspace

# Add agent code
COPY --chown=$USERNAME:$USERNAME ./code /workspace/code/

# Link code into catkin workspace
RUN ln -s /workspace/code /catkin_ws/src

# re-make the catkin workspace
RUN source /opt/ros/noetic/setup.bash && catkin_make

ADD ./build/docker/agent/entrypoint.sh /entrypoint.sh

# set the default working directory to the code
WORKDIR /workspace/code

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

RUN pip install -r requirements.txt

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash", "-c", "sleep 10 && python3 /opt/leaderboard/leaderboard/leaderboard_evaluator.py --debug=${DEBUG_CHALLENGE} \
--repetitions=${REPETITIONS} \
--checkpoint=${CHECKPOINT_ENDPOINT} \
--track=${CHALLENGE_TRACK} \
--agent=${TEAM_AGENT} \
--routes=${ROUTES} \
--host=${CARLA_SIM_HOST}"]
