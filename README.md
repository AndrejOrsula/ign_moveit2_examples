# ign_moveit2_examples

C++ and Python examples of using MoveIt 2 for planning motions that are executed inside Ignition Gazebo simulation environment. These examples make use of [ros2_control](https://github.com/ros-controls/ros2_control) via [ign_ros2_control](https://github.com/ignitionrobotics/ign_ros2_control).

> For legacy approach using [`JointTrajectoryController`](https://github.com/ignitionrobotics/ign-gazebo/blob/ign-gazebo4/src/systems/joint_trajectory_controller/JointTrajectoryController.hh) Ignition plugin, please see [legacy_jtc_ign_plugin](https://github.com/AndrejOrsula/ign_moveit2/tree/legacy_jtc_ign_plugin) branch.

| <img width="100%" src="https://user-images.githubusercontent.com/22929099/147374612-3d0209d3-574e-4a4f-8077-edbbcf8fc47d.gif" alt="Animation of ex_follow_target"/> | <img width="100%" src="https://user-images.githubusercontent.com/22929099/147374613-ad15aa1a-deaf-4dcd-92b0-1a53d0097467.gif" alt="Animation of ex_throw_object"/> |
| :-----------------------------------------------------------------------------------------------------------------------------------------------------------------: | :----------------------------------------------------------------------------------------------------------------------------------------------------------------: |
|                                                                            Follow Target                                                                            |                                                                            Throw Object                                                                            |

At the time of writing these, there are no official Python bindings for MoveIt 2. Therefore, [pymoveit2](https://github.com/AndrejOrsula/pymoveit2) module is employed as the MoveIt 2 interface in all Python examples.

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

Until [ros2_controllers#225](https://github.com/ros-controls/ros2_controllers/pull/225) is merged and released, `ros2_controllers` must be built from source in order to enable the use of effort command interface inside Ignition Gazebo.

- [AndrejOrsula/ros2_controllers:jtc_effort](https://github.com/AndrejOrsula/ros2_controllers/tree/jtc_effort) was tested and can be used for this purpose

Additional dependencies for `pymoveit2` and robot models are listed under [ign_moveit2_examples.repos](./ign_moveit2_examples.repos) and pulled via git during installation. Please, see instructions below.

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
