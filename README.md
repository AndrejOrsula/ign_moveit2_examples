# ign_moveit2

MoveIt2 interface for Ignition Gazebo, in which MoveIt2 is utilised to generate trajectories that are transported across ROS 2<-->Ignition bridge and executed by JointTrajectoryController Ignition plugin.

**Branch note: This might be a temporary location for the JointTrajectoryController plugin.**

![ign_moveit2_communication_scheme](_graphics/ign_moveit2_communication.png)

Examples for control of [Franka Emika Panda](https://github.com/AndrejOrsula/panda_ign.git) inside Ignition Gazebo are included for both C++ and Python.

## Directory Structure

```bash
├── joint_trajectory_controller             # JointTrajectoryController plugin for Ignition
├── moveit2_py                              # Python module for interfacing with MoveIt2 (temporary substitute for moveit_commander)
├── examples                                # Python examples utilising ign_moveit2_py
├── src
    └── ign_moveit2.cpp                     # C++ template/example for interfacing with MoveIt2->Ignition with move_group API
├── launch
    ├── ign_moveit2.launch.py               # Helpful launch file that starts up MoveIt2 move_group action server and bridges between ROS 2 and Ignition
    └── move_group_cpp_example.launch.py    # Launch of the C++ example
├── worlds
    └── panda_example.sdf                   # Barebones example world for Ignition Gazebo
└── ign_moveit2.repos                       # List of other dependencies created for `ign_moveit2`
```

## Instructions

### Dependencies

- [ROS 2 Foxy](https://index.ros.org/doc/ros2/Installation/Foxy)
- [MoveIt2](https://moveit.ros.org/install-moveit2/source)
- [Ignition Dome](https://ignitionrobotics.org/docs/dome/install)
  - For `ign-msgs`, you need to use [AndrejOrsula/ign-msgs - add_joint_trajectory](https://github.com/AndrejOrsula/ign-msgs/tree/add_joint_trajectory) as it contains the required `ign_msgs.JointTrajectory`.

All other dependencies are pulled from git ([ign_moveit2.repos](ign_moveit2.repos)) and built automatically with this repository.

### Building

Clone, clone dependencies and build with `colcon`.

```bash
export PARENT_DIR=${PWD}
mkdir -p ign_moveit2/src && cd ign_moveit2/src
git clone https://github.com/AndrejOrsula/ign_moveit2.git -b joint_trajectory_controller
vcs import < ign_moveit2/ign_moveit2.repos
cd ..
export IGNITION_VERSION=dome
rosdep install --from-paths src -i -y --rosdistro ${ROS_DISTRO}
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

### Usage

#### Environment

Source the ROS 2 workspace overlay.

```bash
source ${PARENT_DIR}/ign_moveit2/install/local_setup.bash
```

Export `IGN_GAZEBO_RESOURCE_PATH` to make SDF of Panda discoverable within the context of Ignition Gazebo.

```bash
export IGN_GAZEBO_RESOURCE_PATH=${PARENT_DIR}/ign_moveit2/src/panda_ign:${IGN_GAZEBO_RESOURCE_PATH}
```

Export `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` to make `JointTrajectoryController` discoverable within the context of Ignition Gazebo.

```bash
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=${PARENT_DIR}/ign_moveit2/install:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}
```

#### Examples

Launch Ignition Gazebo world with Panda robot.

```bash
ign gazebo ${PARENT_DIR}/ign_moveit2/src/ign_moveit2/worlds/panda_example.sdf
```

Start MoveIt2 move_group action server and the required bridges between ROS 2 and Ignition.

```bash
ros2 launch ign_moveit2 ign_moveit2.launch.py
```

Run C++ example.

```bash
ros2 launch ign_moveit2 move_group_cpp_example.launch.py
```

Run Python examples.

```bash
ros2 run ign_moveit2 example_joint_goal.py --ros-args -p use_sim_time:=true
ros2 run ign_moveit2 example_pose_goal.py --ros-args -p use_sim_time:=true
```
