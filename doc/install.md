# Praktikum Autonomes Fahren - Setup

This manual will guide you through the process of setting up everything that is needed for the environment setup.

## Requirements

- **Anaconda** needs to be installed first (<https://www.anaconda.com/products/distribution>)

## Setting up the Anaconda Environment

First you need to create an environment that specifies the Python version that corresponds to the CARLA API

```bash
conda init &&
conda create -y -n paf python=3.7 &&
conda activate paf
```

You should see ```(paf)``` at the beginning of a new terminal.

> **Warning**: You need to execute all of these commands in the same shell or activate the environment if you open a new one!

## Installing ROS Noetic

Next you need to download ROS Noetic and additional dependencies.

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' &&
sudo apt install -y curl &&
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - &&
sudo apt update &&
sudo apt install -y ros-noetic-desktop-full &&
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc &&
source ~/.bashrc &&
conda activate paf &&
sudo apt install -y build-essential git python3-rosdep openssl &&
pip install rosdep rosinstall rosinstall-generator wstool pyyaml empy roslibpy simple_pid pyOpenSSL tornado pymongo &&
sudo rosdep init &&
rosdep update
```

## Installing the CARLA Leaderboard

After that download all components of the CARLA Leaderboard:

- CARLA
- Leaderboard
- Scenario Runner

and configure the corresponding environment variables.

```bash
sudo apt install -y libomp5 &&
wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/Leaderboard/CARLA_Leaderboard_20.tar.gz &&
mkdir -p -m o+w ~/PAF/carla &&
tar -C ~/PAF/carla -zxvf CARLA_Leaderboard_20.tar.gz &&
rm CARLA_Leaderboard_20.tar.gz &&
cd ~/PAF/carla &&
pip install -r PythonAPI/carla/requirements.txt &&
cd .. &&
git clone -b leaderboard-2.0 --single-branch https://github.com/carla-simulator/leaderboard.git &&
cd leaderboard &&
pip install -r requirements.txt &&
cd .. &&
git clone -b leaderboard-2.0 --single-branch https://github.com/carla-simulator/scenario_runner.git &&
cd scenario_runner &&
pip install -r requirements.txt &&
echo "export CARLA_ROOT=/home/${USER}/PAF/carla" >> ~/.bashrc &&
echo "export SCENARIO_RUNNER_ROOT=/home/${USER}/PAF/scenario_runner" >> ~/.bashrc &&
echo "export LEADERBOARD_ROOT=/home/${USER}/PAF/leaderboard" >> ~/.bashrc &&
echo "export PYTHONPATH="/home/${USER}/PAF/carla/PythonAPI/carla/":"/home/${USER}/PAF/scenario_runner":"/home/${USER}/PAF/leaderboard":"/home/${USER}/PAF/carla/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg":${PYTHONPATH}" >> ~/.bashrc &&
source ~/.bashrc
```

## Installing CARLA ROS Bridge

The last component is the CARLA ROS Bridge. Adding the missing build dependency fixes the issue that the ackermann control package is built before its dependency of carla messages.

```bash
cd ~/PAF &&
conda activate paf &&
git clone --recurse-submodules -b leaderboard-2.0 --single-branch https://github.com/carla-simulator/ros-bridge.git &&
cd ros-bridge/carla_ackermann_control &&
sed -i '24 i \  <build_depend condition="$ROS_VERSION == 1">carla_msgs</build_depend>' package.xml &&
source /opt/ros/noetic/setup.bash &&
mkdir -p -m o+w ~/PAF/catkin_ws/src &&
cd ~/PAF/catkin_ws/src &&
ln -s ../../ros-bridge &&
cd .. &&
rosdep update
```

Before proceeding close and reopen the terminal for all changes to take effect.

```bash
cd ~/PAF/catkin_ws &&
conda activate paf &&
rosdep install --from-paths src --ignore-src -r --os=ubuntu:focal &&
catkin_make &&
echo "source /home/${USER}/PAF/catkin_ws/devel/setup.bash" >> ~/.bashrc &&
echo "export CARLA_ROS_BRIDGE_ROOT=/home/${USER}/PAF/ros-bridge" >> ~/.bashrc &&
source ~/.bashrc
conda activate paf
```

## Optional

### Installing Visual Studio Code

We recommend working with Visual Studio Code since it offers a ROS extension for a better workflow.

```bash
sudo snap install --classic code &&
```

### Installing Anydesk for remote access

```bash
sudo -s
sudo wget -qO - https://keys.anydesk.com/repos/DEB-GPG-KEY | sudo apt-key add - &&
sudo echo "deb http://deb.anydesk.com/ all main" > /etc/apt/sources.list.d/anydesk-stable.list &&
sudo apt update &&
sudo apt install -y anydesk
```
