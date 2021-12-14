# ign_moveit2_examples

> For legacy approach using [`JointTrajectoryController`](https://github.com/ignitionrobotics/ign-gazebo/blob/ign-gazebo4/src/systems/joint_trajectory_controller/JointTrajectoryController.hh) Ignition plugin, please see [legacy_jtc_ign_plugin](https://github.com/AndrejOrsula/ign_moveit2/tree/legacy_jtc_ign_plugin) branch.

C++ and Python examples of using MoveIt 2 for planning motions that are executed inside Ignition Gazebo simulation environment. These examples make use of [ros2_control](https://github.com/ros-controls/ros2_control) via [ign_ros2_control](https://github.com/ignitionrobotics/ign_ros2_control).

At the time of writing these, there are no official Python bindings for MoveIt 2. Therefore, [pymoveit2](https://github.com/AndrejOrsula/pymoveit2) module is employed as a temporary solution in all Python examples.

## Instructions

### Requirements

- **OS:** Ubuntu 20.04 (Focal)
  - Other distributions might work (not tested).

### Dependencies

These are the primary dependencies required to use this project.

- ROS 2 [Rolling](https://docs.ros.org/en/rolling/Installation.html)
  - [Galactic](https://docs.ros.org/en/galactic/Installation.html) should also work without any issues (not tested)
- Ignition [Fortress](https://ignitionrobotics.org/docs/fortress)
  - [Citadel](https://ignitionrobotics.org/docs/citadel) and [Edifice](https://ignitionrobotics.org/docs/edifice) should also work (not tested)
- [MoveIt 2](https://moveit.ros.org/install-moveit2/binary)
  - Install/build a version based on the selected ROS 2 release

Furthermore, the following packages are required.

- [ros_ign](https://github.com/ignitionrobotics/ros_ign/tree/ros2)
  - Install/build a version based on the selected combination of ROS 2 release and Ignition version
- [ign_ros2_control](https://github.com/ignitionrobotics/ign_ros2_control)
  - Build a version based on the selected combination of ROS 2 release and Ignition version

Additional dependencies for `pymoveit2` and robot models can be pulled from git ([ign_moveit2_examples.repos](./ign_moveit2_examples.repos)), see instructions below.

### Building

Clone this repository and import VCS dependencies. Then install dependencies and build with [colcon](https://colcon.readthedocs.io).

```bash
# Clone this repository into your favourite ROS 2 workspace
git clone https://github.com/AndrejOrsula/ign_moveit2_examples.git
# Import additional git dependencies
vcs import < ign_moveit2_examples/ign_moveit2_examples.repos
# Install external dependencies via rosdep
rosdep install -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
# Build with colcon
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

### Sourcing

Before utilising this package, remember to source the ROS 2 workspace overlay.

```bash
source ${IGN_MOVEIT2_EXAMPLES_WS_DIR}/install/local_setup.bash
```

This enables:

- Execution of scripts and examples via `ros2 run ign_moveit2_examples <executable>`
- Launching of setup scripts via `ros2 launch ign_moveit2_examples <launch_script>`
- Discoverability of shared resources

## Examples

<!-- TODO -->

### Follow Example

```bash
```

![follow](_graphics/ign_moveit2_follow.gif)

### Throw Example

```bash
```

![throw](_graphics/ign_moveit2_throw.gif)

## Directory Structure

<!-- TODO -->

```bash
.
├── examples
├── launch
└── worlds
```
