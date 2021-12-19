#!/usr/bin/env -S ros2 launch
"""Launch script for spawning Franka Emika Panda into Ignition Gazebo world"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from os import path
from typing import List


def generate_launch_description() -> LaunchDescription:

    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    description_package = LaunchConfiguration("description_package")
    description_filepath = LaunchConfiguration("description_filepath")
    model = LaunchConfiguration("model")
    use_sim_time = LaunchConfiguration("use_sim_time")
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
            model,
        ]
    )
    robot_description = {"robot_description": _robot_description_xml}

    # List of nodes to be launched
    nodes = [
        # robot_state_publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                robot_description,
                {
                    "publish_frequency": 50.0,
                    "frame_prefix": "",
                    "use_sim_time": use_sim_time,
                },
            ],
        ),
        # ros_ign_gazebo_create
        Node(
            package="ros_ign_gazebo",
            executable="create",
            output="log",
            arguments=["-file", model, "--ros-args", "--log-level", log_level],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]

    return LaunchDescription(declared_arguments + nodes)


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
        # Model for Ignition Gazebo
        DeclareLaunchArgument(
            "model",
            default_value="panda",
            description="Name or filepath of model to load.",
        ),
        # Miscellaneous
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
        ),
    ]
