#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from moveit2 import MoveIt2Interface

import time


from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from scipy.spatial.transform import Rotation


class Thrower(Node):

    def __init__(self):
        super().__init__("thrower")
        # Create a subscriber for object pose
        self.object_pose_sub_ = self.create_subscription(Pose, '/model/throwing_object/pose',
                                                         self.object_pose_callback, 1)

        # Create MoveIt2 interface node
        self.moveit2_ = MoveIt2Interface()

        # Create multi-threaded executor
        self.executor_ = rclpy.executors.MultiThreadedExecutor(2)
        self.executor_.add_node(self)
        self.executor_.add_node(self.moveit2_)

        # Wait a couple of seconds until Ignition is ready and spin up the executor
        time.sleep(2)
        self.executor_.spin()

    def object_pose_callback(self, pose_msg):
        self.throw(pose_msg.position)
        self.destroy_subscription(self.object_pose_sub_)
        rclpy.shutdown()

    def throw(self, object_position):
        # Open gripper
        self.moveit2_.gripper_open()
        self.moveit2_.wait_until_executed()

        # Move above object
        position = [object_position.x,
                    object_position.y, object_position.z + 0.1]
        quaternion = Rotation.from_euler('xyz',
                                         [180, 0, 0],
                                         degrees=True).as_quat()
        self.moveit2_.set_pose_goal(position, quaternion)
        self.moveit2_.plan_kinematic_path()
        self.moveit2_.execute()
        self.moveit2_.wait_until_executed()

        self.moveit2_.set_max_velocity(0.25)
        self.moveit2_.set_max_acceleration(0.25)

        # Move to grasp position
        position = [object_position.x,
                    object_position.y, object_position.z]
        quaternion = Rotation.from_euler('xyz',
                                         [180, 0, 0],
                                         degrees=True).as_quat()
        self.moveit2_.set_pose_goal(position, quaternion)
        self.moveit2_.plan_kinematic_path()
        self.moveit2_.execute()
        self.moveit2_.wait_until_executed()

        # Close gripper
        self.moveit2_.gripper_close(width=0.05, speed=0.01, force=20.0)
        self.moveit2_.wait_until_executed()

        # Move above object again
        position = [object_position.x,
                    object_position.y, object_position.z + 0.1]
        quaternion = Rotation.from_euler('xyz',
                                         [180, 0, 0],
                                         degrees=True).as_quat()
        self.moveit2_.set_pose_goal(position, quaternion)
        self.moveit2_.plan_kinematic_path()
        self.moveit2_.execute()
        self.moveit2_.wait_until_executed()

        # Move to pre-throw configuration
        joint_positions = [0.0,
                           -1.0,
                           0.0,
                           -0.4,
                           0.0,
                           3.141,
                           0.785]
        self.moveit2_.set_joint_goal(joint_positions)
        self.moveit2_.plan_kinematic_path()
        self.moveit2_.execute()
        self.moveit2_.wait_until_executed()

        # Throw
        self.moveit2_.set_max_velocity(1.0)
        self.moveit2_.set_max_acceleration(1.0)

        # Arm trajectory
        joint_positions = [0.0,
                           0.8,
                           0.0,
                           -1.57,
                           0.0,
                           3.14,
                           0.785]
        self.moveit2_.set_joint_goal(joint_positions)
        trajectory = self.moveit2_.plan_kinematic_path(
        ).motion_plan_response.trajectory.joint_trajectory

        # Hand opening trajectory
        hand_trajectory = self.moveit2_.gripper_plan_path(0.08, 0.2)

        # Merge hand opening into arm trajectory, such that it is times for release (at 50%)
        release_index = round(0.5*len(trajectory.points))
        for finger_joint in hand_trajectory.joint_names:
            trajectory.joint_names.append(finger_joint)
        while len(trajectory.points[release_index].effort) < 9:
            trajectory.points[release_index].effort.append(0.0)
        for finger_index in range(2):
            trajectory.points[release_index].positions.append(
                hand_trajectory.points[-1].positions[finger_index])
            trajectory.points[release_index].velocities.append(
                hand_trajectory.points[-1].velocities[finger_index])
            trajectory.points[release_index].accelerations.append(
                hand_trajectory.points[-1].accelerations[finger_index])

        self.moveit2_.execute(trajectory)
        self.moveit2_.wait_until_executed()

        # Move to default position
        joint_positions = [0.0,
                           0.0,
                           0.0,
                           -1.57,
                           0.0,
                           1.57,
                           0.79]
        self.moveit2_.set_joint_goal(joint_positions)
        self.moveit2_.plan_kinematic_path()
        self.moveit2_.execute()
        self.moveit2_.wait_until_executed()

        rclpy.shutdown()
        exit(0)


def main(args=None):
    rclpy.init(args=args)

    _thrower = Thrower()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
