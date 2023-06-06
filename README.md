# ign_moveit2_examples

C++ and Python examples of using MoveIt 2 for planning motions that are executed inside ~~Ignition~~ Gazebo simulation environment. These examples make use of [ros2_control](https://github.com/ros-controls/ros2_control) via [gz_ros2_control](https://github.com/ros-controls/gz_ros2_control).

The interaction among different ROS 2, MoveIt 2 and Gazebo components is further documented in [docs/README.md](./docs/README.md), alongside suggestions for implementing a similar setup on your custom robot.

> For legacy approach using [`JointTrajectoryController`](https://github.com/ignitionrobotics/ign-gazebo/blob/ign-gazebo4/src/systems/joint_trajectory_controller/JointTrajectoryController.hh) Ignition plugin, please see [legacy_jtc_ign_plugin](https://github.com/AndrejOrsula/ign_moveit2/tree/legacy_jtc_ign_plugin) branch.

| <img width="100%" src="https://user-images.githubusercontent.com/22929099/147374612-3d0209d3-574e-4a4f-8077-edbbcf8fc47d.gif" alt="Animation of ex_follow_target"/> | <img width="100%" src="https://user-images.githubusercontent.com/22929099/147374613-ad15aa1a-deaf-4dcd-92b0-1a53d0097467.gif" alt="Animation of ex_throw_object"/> |
| :-----------------------------------------------------------------------------------------------------------------------------------------------------------------: | :----------------------------------------------------------------------------------------------------------------------------------------------------------------: |
|                                                                            Follow Target                                                                            |                                                                            Throw Object                                                                            |

At the time of writing these examples, there were no official Python bindings for MoveIt 2. Therefore, [pymoveit2](https://github.com/AndrejOrsula/pymoveit2) module is employed as the MoveIt 2 interface in all Python examples.

## Instructions

### Dependencies

These are the primary dependencies required to use this project.

- ROS 2 [Galactic](https://docs.ros.org/en/galactic/Installation.html)
- Gazebo [Fortress](https://gazebosim.org/docs/fortress)

All additional dependencies are either pulled via [vcstool](https://wiki.ros.org/vcstool) ([ign_moveit2_examples.repos](./ign_moveit2_examples.repos)) or installed via [rosdep](https://wiki.ros.org/rosdep) during the building process below.

### Building

Clone this repository, import dependencies, install dependencies and build with [colcon](https://colcon.readthedocs.io).

```bash
# Clone this repository into your favourite ROS 2 workspace
git clone https://github.com/AndrejOrsula/ign_moveit2_examples.git
# Import dependencies
vcs import < ign_moveit2_examples/ign_moveit2_examples.repos
# Install dependencies
IGNITION_VERSION=fortress rosdep install -y -r -i --rosdistro ${ROS_DISTRO} --from-paths .
# Build
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

### Sourcing

Before utilising this package, remember to source the ROS 2 workspace.

```bash
source install/local_setup.bash
```

This enables:

- Execution of binaries, scripts and examples via `ros2 run ign_moveit2_examples <executable>`
- Launching of setup scripts via `ros2 launch ign_moveit2_examples <launch_script>`
- Discoverability of shared resources

## Examples

In order to run any of the included examples, just launch the corresponding script.

### Follow Target

```bash
# C++
ros2 launch ign_moveit2_examples ex_cpp_follow_target.launch.py
# Python
ros2 launch ign_moveit2_examples ex_py_follow_target.launch.py
```

### Throw Object

```bash
# Python
ros2 launch ign_moveit2_examples ex_py_throw_object.launch.py
```

## Directory Structure

```bash
.
├── examples/                      # [dir] Nodes used for examples
    ├── cpp/                       # [dir] C++ nodes
    └── py/                        # [dir] Python nodes
├── launch/                        # [dir] Launch scripts for examples
    ├── robots/                    # [dir] Launch scripts that spawn robots into environment
    ├── worlds/                    # [dir] Launch scripts that setup the environment
    ├── default.launch.py          # Default launch script used by all edxamples
    ├── ex_cpp_*.launch.py         # C++ launch scripts
    └── ex_py_*.launch.py          # Python launch scripts
├── rviz/ign_moveit2_examples.rviz # RViz2 config for motion planning with MoveIt 2
├── worlds/                        # [dir] World descriptors
├── CMakeLists.txt                 # Colcon-enabled CMake recipe
└── package.xml                    # ROS 2 package metadata
```
