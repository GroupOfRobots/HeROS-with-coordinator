# HeROS with STM-based coordinator
> **Note**
> In this case, the multi-robot system (MRS) has a central architecture - a finite state machine was used to allocate tasks to the robots.

## Description
The goal of this project is to develop methods of cooperation between manipulators and mobile robots using the ROS 2 framework. MiniRyś mobile robot transports objects between two Dobot Magician manipulators - one of them loads the object onto the mobile robot and the other takes the object off at the unloading point.

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
git clone https://github.com/GroupOfRobots/HeROS-with-coordinator
cd ..
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```
## Running
Run node with FSM:
```bash
ros2 run heros_test_scenario cubes_transport
```

In another terminal, run _YASMIN Viewer_ so you can monitor the state of the FSM:
```bash
ros2 run yasmin_viewer yasmin_viewer_node
```

In your web browser open http://localhost:5000/. In the checkbox at the top of the page, change _ALL_ to _HEROS\_SCENARIO_ to enlarge the FSM visualization. 
