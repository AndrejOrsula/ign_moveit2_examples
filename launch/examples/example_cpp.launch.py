"""Launch example of using C++ move_group interface to control robot in Ignition"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    config_rviz2 = LaunchConfiguration('config_rviz2', default=os.path.join(get_package_share_directory("ign_moveit2"),
                                                                            "launch", "rviz.rviz"))

    # URDF
    robot_urdf_config = load_file("panda_ign",
                                  "urdf/panda.urdf")
    robot_description = {"robot_description": robot_urdf_config}

    # SRDF
    robot_srdf = load_file("panda_moveit2_config",
                           "srdf/panda.srdf")
    robot_description_semantic = {"robot_description_semantic": robot_srdf}

    # Kinematics
    kinematics = load_yaml("panda_moveit2_config",
                           "config/kinematics.yaml")

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        DeclareLaunchArgument(
            'config_rviz2',
            default_value=config_rviz2,
            description='Path to config for RViz2'),

        # C++ example executable
        Node(package='ign_moveit2',
             executable='example_ign_moveit2',
             name='example_ign_moveit2',
             output='screen',
             parameters=[robot_description,
                         robot_description_semantic,
                         kinematics,
                         {'use_sim_time': use_sim_time}])
    ])
