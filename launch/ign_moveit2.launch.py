"""Launch MoveIt2 move_group action server and the required bridges between Ignition and ROS 2"""

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

        # MoveIt2 move_group action server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory("panda_moveit2_config"),
                              "launch", "move_group_action_server.launch.py")]),
            # Simulation time does not function properly (as of Nov 2020), see https://github.com/AndrejOrsula/ign_moveit2/issues/4
            launch_arguments=[('use_sim_time', "False"),
                              ('config_rviz2', config_rviz2)]),

        # Clock bridge (IGN -> ROS2)
        Node(package="ros_ign_bridge",
             executable="parameter_bridge",
             name="parameter_bridge_block",
             output="screen",
             arguments=[
                 "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
             parameters=[{'use_sim_time': use_sim_time}]),

        # JointState bridge (IGN -> ROS2)
        Node(package="ros_ign_bridge",
             executable="parameter_bridge",
             name="parameter_bridge_joint_states",
             output="screen",
             arguments=[
                 "/world/default/model/panda/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model"],
             parameters=[{'use_sim_time': use_sim_time}],
             remappings=[("/world/default/model/panda/joint_state", "/joint_states")]),

        # JointTrajectory bridge (ROS2 -> IGN)
        Node(package="ros_ign_bridge",
             executable="parameter_bridge",
             name="parameter_bridge_joint_trajectory",
             output="screen",
             arguments=[
                 "/joint_trajectory@trajectory_msgs/msg/JointTrajectory]ignition.msgs.JointTrajectory"],
             parameters=[{'use_sim_time': use_sim_time}]),

        # JointTrajectoryProgress bridge (IGN -> ROS2)
        Node(package="ros_ign_bridge",
             executable="parameter_bridge",
             name="parameter_bridge_joint_trajectory_progreess",
             output="screen",
             arguments=[
                 "/joint_trajectory_progress@std_msgs/msg/Float32[ignition.msgs.Float"],
             parameters=[{'use_sim_time': use_sim_time}])
    ])
