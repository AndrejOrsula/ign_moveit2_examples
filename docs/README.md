# Example documentation

This document describes the different software components
used by the MoveIt 2, ROS 2, Ignition Gazebo demo,
where to find them and how they are being called.

***Note:*** only the follow target is used in this document

The document details:

 1. Code repositories being used in the demo
 1. How to prepare a Robot URDF for using Gazebo with ros2_control
 1. How to connect MoveIt 2 with the Gazebo ros2_control plugins
 1. How this example declares the target pose for the manipulation
 1. Other important ROS2 packages for the simulation
 1. Graph of runtime ROS 2 topics and nodes

## Code repositories used in the demo 

Repositories used:

* [Panda Ign MoveIt 2](https://github.com/AndrejOrsula/panda_ign_moveit2):
  Software packages for Franka Emika Panda Robot that enable manipulation with MoveIt 2 
  inside Ignition Gazebo. For control, ignition_ros2_control is used.

* [ROS Ignition Gazebo](https://github.com/gazebosim/ros_gz/tree/galactichttps://github.com/gazebosim/ros_gz/tree/galactic)
  This repository holds packages that provide integration between ROS and Ignition.
  Mainly `ros_ign_package` is being used to launch Gazebo with ROS 2 integration 
  and `ros_ign_bridge` to convert Gz messages to ROS messages.

* [Igition Gazebo ROS2 Control](https://github.com/ros-controls/gazebo_ros2_control/tree/galactic)
  This is a ROS 2 package for integrating the ros2_control controller architecture
  with the Gazebo simulator.

### How to prepare a Robot URDF for using Gazebo with ros2_control

Two main actions needs to be done to prepare the URDF model to use Gazebo and
ros2_control, configure ros2_control setting using the `ros2_control` URDF
macro and add a plugin to Gazebo to parses the `ros2_control` tags and 
loads the appropriate hardware interfaces and controller manager

#### 1. URDF declaration for the [`ros2_control` URDF tag](https://control.ros.org/master/doc/getting_started/getting_started.html#hardware-description-in-urdf)

This is done by adding the xacro macro `ros2_control_panda_arm` (which is 
integrated from main Panda arm URDF file) and uses the
[`ign_ros2_control/IgnitionSystem`](https://github.com/ros-controls/gz_ros2_control/blob/master/README.md?plain=1#L93-L118):

* https://github.com/AndrejOrsula/panda_ign_moveit2/blob/master/panda_description/urdf/panda.ros2_control#L28-L30

#### 2. URDF declaration for the Gazebo plugin [IgnitionROS2ControlPlugin](https://github.com/ros-controls/gz_ros2_control/blob/master/README.md?plain=1#L153-L169)

This is done by adding the xacro macro `ign_ros2_control` (which is integrated 
from main Panda URDF file) and add the Gazebo plugin `IgnitionROS2ControlPlugin`
that parses the `ros2_control` tags and loads the appropriate hardware interfaces and controller manager

* https://github.com:drejOrsula/panda_ign_moveit2/blob/master/panda_description/urdf/panda.gazebo#L8-L16

### How to connect MoveIt 2 with the Gazebo ros2_control plugins

The example launches MoveIt using igniton_ros2_control:

* [default.launch.py](../launch/default.launch.py#L81-L94)

This snippet is calling the main [Panda MoveIt configuration launch file](https://github.com/AndrejOrsula/panda_ign_moveit2/blob/master/panda_moveit_config/launch/move_group.launch.py):

* Panda main MoveIt 2 move_group invocation:
  * https://github.com/AndrejOrsula/panda_ign_moveit2/blob/5679c0a97d5e34199271ed7abf02cf303d0e8fb9/panda_moveit_config/launch/move_group.launch.py#L248-L264

* Controller manager
  * https://github.com/AndrejOrsula/panda_ign_moveit2/blob/5679c0a97d5e34199271ed7abf02cf303d0e8fb9/panda_moveit_config/launch/move_group.launch.py#L309-L324

### How this example declares the target pose for the manipulation

This repository uses a `Gazebo PosePublisher` plugin to indicate the target pose for the manipulation:
* [follow_target.sdf](../ign_moveit2_examples/worlds/follow_target.sdf#L99-L104)

Translate the pose from Gazebo to ROS using the `ros_ign_bridge`:
* [world_follow_target.launch.py](../launch/worlds/world_follow_target.launch.py#L61-L76)

The /target_pose is subscribed to by the ROS C++ node that communicates to MoveIt
* [ex_follow_target.cpp](../examples/cpp/ex_follow_target.cpp#L28-L57)

### Other important ROS 2 packages

#### Robot state publisher

The Robot State Publisher is a node and a class to publish
the state of a robot to tf2. At startup time, Robot State Publisher is 
supplied with a kinematic tree model (URDF) of the robot. It then subscribes 
to the joint_states topic (of type sensor_msgs/msg/JointState) to get
individual joint states. More information in the
[ROS2 tutorial](https://docs.ros.org/en/galactic/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher.html)

* https://github.com/ros/robot_state_publisher

#### TF2 

tf2 is the second generation of the transform library, which lets the user
keep track of multiple coordinate frames over time. tf2 maintains the 
relationship between coordinate frames in a tree structure buffered in 
time, and lets the user transform points, vectors, etc between any 
two coordinate frames at any desired point in time.

* http://wiki.ros.org/tf2


## ROS 2 Topics and Nodes

![ROS 2 Graph of the demo](ros_rqt_graph.png)

Colors legend for nodes:
 * Green: Gazebo special nodes for ROS integration
 * Purple: pose target created by the own example code in this repo
 * Blue: nodes create and handle by `ros2_control` invoked from `ignition_ros_control`
 * Yellow: `moveit2` nodes
 * Red: other ROS 2 important nodes usually needed for simulations
