# HeROS - test scenario

## Description
The goal of the project is to develop methods of cooperation between manipulators and mobile robots using the ROS 2 framework. MiniRyś mobile robot transports objects between two Dobot Magician manipulators - one of them loads the object onto the robot and the other takes the object off at the unloading point. In this case, the multi-robot system (MRS) has a central architecture - a finite state machine was used to allocate tasks to the robots

As part of the project, we used YASMIN. It is a project focused on implementing robot behaviors using Finite State Machines (FSM). It is available for ROS 2, Python and C++. For more information, see:
* [**YASMIN GitHub repository**](https://github.com/uleroboticsgroup/yasmin)
* [**YASMIN: Yet Another State MachINe library for ROS 2 (article posted on arXiv)**](https://arxiv.org/abs/2205.13284)

## Installation and building

### Install YASMIN

```bash
# Create ROS 2 workspace and clone repositories
mkdir -p ~/yasmin_ws/src/
cd ~/yasmin_ws/src/
git clone https://github.com/uleroboticsgroup/simple_node.git
git clone https://github.com/uleroboticsgroup/yasmin.git

# Install dependencies
cd yasmin
pip3 install -r requirements.txt
```

### Clone HeROS-use-case repository and build workspace
```bash
cd ~/yasmin_ws/src/
git clone https://github.com/RCPRG-ros-pkg/HeROS-use-case.git
cd ..
source /opt/humble/setup.bash
colcon build
```
## Running
