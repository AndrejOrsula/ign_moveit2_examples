# ign_moveit2

Examples of using [MoveIt2](https://moveit.ros.org) for planning motions that are executed inside [Ignition Gazebo](https://ignitionrobotics.org) simulation environment. These examples do NOT make use of [ros2_control](https://github.com/ros-controls/ros2_control)\*, but instead employ [`JointTrajectoryController`](https://github.com/ignitionrobotics/ign-gazebo/blob/ign-gazebo4/src/systems/joint_trajectory_controller/JointTrajectoryController.hh) Ignition system plugin with the communication graph illustrated below. All communication and `move_group` action server can be setup using [`ign_moveit2.launch.py`](launch/ign_moveit2.launch.py), which then allows usage with both C++ and Python interface.

![ign_moveit2_communication_scheme](_graphics/ign_moveit2_communication.svg)

This repository previously contained also `JointTrajectoryController` which is now merged into `ign-gazebo`. Therefore, this repository should serve as a template/example of using such setup and no longer needs to be installed.

To demonstrate Python usage, a minimal [Python MoveIt2 interface](moveit2_py/moveit2.py) is also provided in this repository for [Franka Emika Panda](https://github.com/AndrejOrsula/panda_ign.git) because `moveit_commander` is not yet ported as of writing this ([tracker](https://github.com/ros-planning/moveit2/issues/314)). Once Python bindings for MoveIt2 are migrated, it is recommended to use those instead.

\* Once `ros2_control` is ready to use (which it might already be at the time of reading this), then developing and using `ign_ros2_control` similar to [gazebo_ros2_control](https://github.com/ros-simulation/gazebo_ros2_control) might be recommended over the solution proposed in this repository.

## Directory Structure

```bash
├── launch
    ├── ign_moveit2.launch.py    # Helpful launch file that starts up MoveIt2 move_group action server and bridges between ROS 2 and Ignition
    └── examples                 # Launch files for examples
├── examples                     # C++ and Python examples
├── moveit2_py                   # Python module for interfacing with MoveIt2 (temporary substitute for moveit_commander)
├── worlds
    ├── panda_follow.sdf         # Gazebo world for follow examples
    └── panda_throw.sdf          # Gazebo world for throw example
└── panda.repos                  # List of additional dependencies for `Franka Emika Panda`
```

## Dependencies

All examples were tested with the following setup.

- ROS 2 [Foxy](https://docs.ros.org/en/foxy/Installation.html) OR [Rolling](https://docs.ros.org/en/rolling/Installation.html)
- Ignition [Dome](https://ignitionrobotics.org/docs/dome/install) OR [Fortress (recommended)](https://ignitionrobotics.org/docs/fortress)
- [MoveIt 2](https://moveit.ros.org/install-moveit2/binary)
  - Install/build based on the selected ROS 2 release
- [ros_ign](https://github.com/ignitionrobotics/ros_ign/tree/ros2)
  - Install/build release based on the selected combination of ROS 2 release and Ignition version

Additional robot-specific dependencies are also required, e.g. [panda.repos](panda.repos).

## Examples

### Building

This step is required to install [Python MoveIt2 interface](moveit2_py/moveit2.py) and additional robot-specific dependencies for Franka Emika Panda in order to try out the examples.

The recommended way is to clone this repository, clone dependencies and build with `colcon`.

```bash
export PARENT_DIR=${PWD}
mkdir -p ign_moveit2/src && cd ign_moveit2/src
git clone https://github.com/AndrejOrsula/ign_moveit2.git
vcs import < ign_moveit2/panda.repos
cd ..
rosdep install --from-paths src -i -y --rosdistro ${ROS_DISTRO}
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

### Environment

First, remember to source the ROS 2 workspace overlay before trying out any of the examples below.

```bash
source ${PARENT_DIR}/ign_moveit2/install/local_setup.bash
```

#### Follow Example

```bash
ros2 launch ign_moveit2 example_follow_object.launch.py
```

![follow](_graphics/ign_moveit2_follow.gif)

#### Throw Example

```bash
ros2 launch ign_moveit2 example_throw.launch.py
```

![throw](_graphics/ign_moveit2_throw.gif)
