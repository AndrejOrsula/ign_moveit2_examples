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
        self.object_pose_sub_ = self.create_subscription(Pose, '/model/throwing_box/pose',
                                                         self.object_pose_callback, 1)

        # Create MoveIt2 interface node
        self.moveit2_ = MoveIt2Interface()

        # Spin up multi-threaded executor
        self.executor_ = rclpy.executors.MultiThreadedExecutor(2)
        self.executor_.add_node(self)
        self.executor_.add_node(self.moveit2_)
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

        # # Close gripper
        self.moveit2_.gripper_close(width=0.03, speed=0.05, force=20.0)
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

        # Move to pre-throw configuration
        joint_positions = [0.0,
                           -0.5,
                           0.0,
                           0.0,
                           0.0,
                           3.1415927,
                           0.0]
        self.moveit2_.set_joint_goal(joint_positions)
        self.moveit2_.plan_kinematic_path()
        self.moveit2_.execute()
        self.moveit2_.wait_until_executed()


        # Throw
        joint_state = JointState()
        joint_state.name = self.moveit2_.arm_joints
        joint_state.position = [0.0,
                                0.5,
                                0.0,
                                -0.39269908,
                                0.0,
                                3.5415927,
                                0.0]
        self.moveit2_.move_to_joint_state(joint_state,
                                          set_position=True,
                                          set_velocity=False,
                                          set_effort=False)
        
        time.sleep(0.7)

        joint_state = JointState()
        joint_state.name = self.moveit2_.gripper_joints
        joint_state.position = [0.04,
                                0.04]
        joint_state.effort = [0.0,
                              0.0]
        self.moveit2_.move_to_joint_state(joint_state,
                                          set_position=True,
                                          set_velocity=False,
                                          set_effort=True)


def main(args=None):
    rclpy.init(args=args)

    _thrower = Thrower()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
