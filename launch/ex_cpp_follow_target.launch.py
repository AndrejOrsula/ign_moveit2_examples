#!/usr/bin/env -S ros2 launch
"""Launch C++ example for following a target"""

from os import path
from typing import List

import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def generate_launch_description() -> LaunchDescription:

    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    description_package = LaunchConfiguration("description_package")
    description_filepath = LaunchConfiguration("description_filepath")
    moveit_config_package = "panda_moveit_config"
    robot_type = LaunchConfiguration("robot_type")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    ign_verbosity = LaunchConfiguration("ign_verbosity")
    log_level = LaunchConfiguration("log_level")

    # URDF
    _robot_description_xml = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), description_filepath]
            ),
            " ",
            "name:=",
            robot_type,
        ]
    )
    robot_description = {"robot_description": _robot_description_xml}

    # SRDF
    _robot_description_semantic_xml = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(moveit_config_package),
                    "srdf",
                    "panda.srdf.xacro",
                ]
            ),
            " ",
            "name:=",
            robot_type,
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": _robot_description_semantic_xml
    }

    # List of included launch descriptions
    launch_descriptions = [
        # Launch world with robot (configured for this example)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("ign_moveit2_examples"),
                        "launch",
                        "default.launch.py",
                    ]
                )
            ),
            launch_arguments=[
                ("world_type", "follow_target"),
                ("robot_type", robot_type),
                ("rviz_config", rviz_config),
                ("use_sim_time", use_sim_time),
                ("ign_verbosity", ign_verbosity),
                ("log_level", log_level),
            ],
        ),
    ]

    # List of nodes to be launched
    nodes = [
        # Run the example node (C++)
        Node(
            package="ign_moveit2_examples",
            executable="ex_follow_target",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                robot_description,
                robot_description_semantic,
                {"use_sim_time": use_sim_time},
            ],
        ),
    ]

    return LaunchDescription(declared_arguments + launch_descriptions + nodes)


def load_yaml(package_name: str, file_path: str):
    """
    Load yaml configuration based on package name and file path relative to its share.
    """

    package_path = get_package_share_directory(package_name)
    absolute_file_path = path.join(package_path, file_path)
    return parse_yaml(absolute_file_path)


def parse_yaml(absolute_file_path: str):
    """
    Parse yaml from file, given its absolute file path.
    """

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        # Locations of robot resources
        DeclareLaunchArgument(
            "description_package",
            default_value="panda_description",
            description="Custom package with robot description.",
        ),
        DeclareLaunchArgument(
            "description_filepath",
            default_value=path.join("urdf", "panda.urdf.xacro"),
            description="Path to xacro or URDF description of the robot, relative to share of `description_package`.",
        ),
        # Robot selection
        DeclareLaunchArgument(
            "robot_type",
            default_value="panda",
            description="Name of the robot to use.",
        ),
        # Miscellaneous
        DeclareLaunchArgument(
            "rviz_config",
            default_value=path.join(
                get_package_share_directory("ign_moveit2_examples"),
                "rviz",
                "ign_moveit2_examples.rviz",
            ),
            description="Path to configuration for RViz2.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "ign_verbosity",
            default_value="2",
            description="Verbosity level for Ignition Gazebo (0~4).",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
        ),
    ]
