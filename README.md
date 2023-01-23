[![Humble-Ubuntu-Latest](https://github.com/miccol/bt_use_cases/actions/workflows/humble-ubuntu.yml/badge.svg)](https://github.com/miccol/bt_use_cases/actions/workflows/humble-ubuntu.yml)

# BT Use Cases
A set of use cases of Behavior Trees in ROS2 and C++.
In this repo, we will use:
- The [Groot](https://github.com/BehaviorTree/Groot) graphical editor di design the BTs.
- The [BehaviorTree.CPP](https://www.behaviortree.dev/) library to execute the BTs.
- The [Nav2](https://navigation.ros.org/index.html)
- Some of the design principles of the book [Behavior Trees in Robotics and AI](https://arxiv.org/abs/1709.00084).
- Some of the nomenclature from [RobMoSys](https://robmosys.eu/), Composable Models and Software for Robotics Systems.
- Some utility functions from base classes from [ros2_bt_utils](https://github.com/miccol/ros2_bt_utils).


> **Note**
> This is not a tutorial on [BehaviorTree.CPP](https://www.behaviortree.dev/), please see the tutorial of the library first. You may want to take a look at the
[TurtleBot3 Behavior Demos](https://github.com/sea-bass/turtlebot3_behavior_demos) on demos on BTs nodes implementation as well.
The use cases below focus on the BT design and the overall software structure.


## Dependencies
- [ROS2 (Humble)](http://docs.ros.org/en/humble/)
- [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP)
- [Nav2](https://navigation.ros.org/index.html) 
- [ros2_bt_utils](https://github.com/miccol/ros2_bt_utils)
- [Groot](https://github.com/BehaviorTree/Groot) (Optional)
- [ZeroMQ](https://zeromq.org/download) (Optional)
- [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) 

## Installation

1.0 - Install the dependencies above
```bash 
sudo apt install libgazebo11 python3-pip
pip install transforms3d 
sudo apt install libzmq3-dev libboost-dev 
sudo apt install qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev
sudo apt install ros-humble-gazebo-* 
sudo apt install ros-humble-behaviortree-cpp-v3 
sudo apt install ros-humble-navigation2 
sudo apt install ros-humble-nav2-bringup 
```
1.1 - pull and install the dependencies for these repos in your ROS2 workspace
```bash 
cd <your-ws>
git clone https://github.com/miccol/ros2_bt_utils.git 
git clone https://github.com/miccol/bt_use_cases.git 
git clone https://github.com/BehaviorTree/Groot.git 
cd .. 
rosdep install --from-paths src --ignore-src 
```


2 - pull this repo in your ROS2 workspace
```bash 
 cd <your-ws>
 git clone https://github.com/miccol/bt_use_cases
```
3 - build

```bash
colcon build
```

Don't forget to export the correct environmental variable for the simulation
```bash 
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models
```

## Overall Software Structure
Each demo has the following folders:

- `bts/descriptions`: that contains the XML files that describe the BT.
- `bts/src`: that contains the source code for custom-made BT leaf nodes and the main executable of the BTs for the demo.
- `components`: that contains the source code for ROS nodes that provide the functionality to the system through formally defined services (e.g. in ROS are [Topics, Services, and Actions](https://docs.ros.org/en/humble/How-To-Guides/Topics-Services-Actions.html)).
- `interfaces`: that contains the custom interface descriptions (e.g. `.msg` for ROS messages, `.srv` for ROS services, `.action` for ROS Actions, etc) of the use case.

#### Why this structure?

We follow separation of concerns (and common sense) by writing in the *components* the code to implement the services required for the task (acting as the server side of the service), in the *bts* the code for the leaf nodes, that usually handles the requests and responses for services  (acting as the client side of the service), and in the `description` the orchestration of the leaf nodes. 
Following this design, the code inside a leaf node becomes (hopefully) neat. Moreover, the behavior designer can test their BTs by implementing fake components with the same interface as the real ones. As done in the Use Cases below.
Over the years, I found this separation quite useful.

The figure below depicts a schema with the two actors from the [RobMoSys's Ecosystem Organization](https://robmosys.eu/wiki/general_principles:ecosystem:start) that are mainly involved in this design: The [Behavior Developer](https://robmosys.eu/wiki/general_principles:ecosystem:roles:behavior_developer) and the [Component Supplier](https://robmosys.eu/wiki/general_principles:ecosystem:roles:component_supplier) 

<p align="center">
  <img src="https://user-images.githubusercontent.com/8132627/195653872-60b4380f-7aaa-4e0a-95e3-3497ec47a97d.png" />
</p>

which, in ROS2 can translate to:

<p align="center">
  <img src="https://user-images.githubusercontent.com/8132627/191484440-188a3203-7486-4983-8f2d-268e390adb58.jpg" />
</p>


 **To recap**
 BTs orchestrate leaf nodes, leaf nodes orchestrate services of components, and the components do the actual job. 


## BT Design Principles

With little surprise, I follow the Design Principles of the book [Behavior Trees in Robotics and AI](https://arxiv.org/pdf/1709.00084.pdf) in particular, the ones in the following chapters:
 
- 3.1 Improving Readability using Explicit Success Conditions
- 3.4 Improving Safety using Sequences
- 3.5 Creating Deliberative BTs using Backchaining

## Use Cases

Below you can find the list of use cases (click on the links)

> **Note**
> For educational purposes, I duplicated some files so that each Use Case is self-contained. 
> 
### [Use Case 0: Simple Robot](simple_example/)

### [Use Case 1: Tour Robot](tour_robot/)

### More To Come
