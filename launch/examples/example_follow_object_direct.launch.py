"""Launch example (Python) of following an object with direct commands for PID joint controllers"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    config_rviz2 = LaunchConfiguration('config_rviz2', default=os.path.join(get_package_share_directory("ign_moveit2"),
                                                                            "launch", "rviz.rviz"))

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

        # MoveIt2 move_group action server with necessary ROS2 <-> Ignition bridges
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory("ign_moveit2"),
                              "launch", "ign_moveit2.launch.py")]),
            launch_arguments=[('use_sim_time', use_sim_time),
                              ('config_rviz2', config_rviz2)]),

        # Launch world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory("ign_moveit2"),
                              "launch", "examples", "worlds", "world_panda_follow.launch.py")]),
            launch_arguments=[('use_sim_time', use_sim_time)]),

        # Python example script (simple object follower)
        Node(name='ign_moveit2_example_follow_object_direct',
             package='ign_moveit2',
             executable='example_follow_object_direct.py',
             output='screen',
             parameters=[{'use_sim_time': use_sim_time}])
    ])
