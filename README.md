# LynxMotion AL5D

A high and low level driver to control the LynxMotion AL5D robot arm using ROS2. 

## Table of Contents

- [LynxMotion AL5D](#lynxmotion-al5d)
  - [Table of Contents](#table-of-contents)
- [Requirements](#requirements)
- [Installation](#installation)
  - [ROS2 Foxy](#ros2-foxy)
  - [Setting up the project](#setting-up-the-project)
- [Building](#building)
- [Running](#running)
  - [Terminal 1: Robotarm program](#terminal-1-robotarm-program)
  - [Terminal 2: Demo program](#terminal-2-demo-program)

# Requirements
 * Ubuntu Linux - Focal Fossa (20.04)
 * ROS2 Foxy

# Installation

## ROS2 Foxy

ROS2 Foxy can be installed using the following commands:

```
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-foxy-desktop
```

More information can be found in the [official documentation](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html).

## Setting up the project

Clone this project into a new ROS2 workspace as follows:

```
mkdir -p ~/robotarm/src
cd ~/robotarm/src
git clone https://github.com/wilricknl/AL5D.git
```

# Building

Once set up, the project can be built with the following commands:

```
cd ~/robotarm
source /opt/ros/foxy/setup.bash
rosdep install -i --from-path src --rosdistro foxy -y
colcon build --packages-select robotarm
```

# Running

Running the demo and robotarm programming requires two terminals. Besides,
the laptop should be connected to the LynxMotion AL5D robotic arm using USB.

## Terminal 1: Robotarm program

The robot program can be run using the following commands:

```
cd ~/robotarm
source /opt/ros/foxy/setup.bash
. install/setup.bash
ros2 run robotarm robot
```

## Terminal 2: Demo program

The demo program can be run using the following commands:

```
cd ~/robotarm
source /opt/ros/foxy/setup.bash
. install/setup.bash
ros2 run robotarm demo
```
