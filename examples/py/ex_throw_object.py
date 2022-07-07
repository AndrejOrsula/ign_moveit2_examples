#!/usr/bin/env python3

from copy import deepcopy
from threading import Thread

import rclpy
from geometry_msgs.msg import Point, Quaternion
from pymoveit2 import MoveIt2, MoveIt2Gripper
from pymoveit2.robots import panda
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node


class MoveItThrowObject(Node):
    def __init__(self):

        super().__init__("ex_throw_object")

        # Create callback group that allows execution of callbacks in parallel without restrictions
        self._callback_group = ReentrantCallbackGroup()

        # Create MoveIt 2 interface
        self._moveit2 = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_ARM,
            callback_group=self._callback_group,
        )
        # Use upper joint velocity and acceleration limits
        self._moveit2.max_velocity = 1.0
        self._moveit2.max_acceleration = 1.0

        # Create MoveIt 2 interface for gripper
        self._moveit2_gripper = MoveIt2Gripper(
            node=self,
            gripper_joint_names=panda.gripper_joint_names(),
            open_gripper_joint_positions=panda.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=panda.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=panda.MOVE_GROUP_GRIPPER,
            callback_group=self._callback_group,
        )

        self.get_logger().info("Initialization successful.")

    def throw(self):
        """
        Plan and execute hard-coded trajectory with intention to throw an object
        """

        self.get_logger().info("Throwing... Wish me luck!")

        throwing_object_pos = Point(x=0.5, y=0.0, z=0.015)
        default_quat = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

        # Open gripper
        self._moveit2_gripper.open()
        self._moveit2_gripper.wait_until_executed()

        # Move above object
        position_above_object = deepcopy(throwing_object_pos)
        position_above_object.z += 0.15
        self._moveit2.move_to_pose(
            position=position_above_object,
            quat_xyzw=default_quat,
        )
        self._moveit2.wait_until_executed()

        # Move to grasp position
        self._moveit2.move_to_pose(
            position=throwing_object_pos,
            quat_xyzw=default_quat,
        )
        self._moveit2.wait_until_executed()

        # Close gripper
        self._moveit2_gripper.close()
        self._moveit2_gripper.wait_until_executed()

        # Decrease speed
        self._moveit2.max_velocity = 0.25
        self._moveit2.max_acceleration = 0.1

        # Move above object (again)
        self._moveit2.move_to_pose(
            position=position_above_object,
            quat_xyzw=default_quat,
        )
        self._moveit2.wait_until_executed()

        # Move to pre-throw configuration
        joint_configuration_pre_throw = [0.0, -1.75, 0.0, -0.1, 0.0, 3.6, 0.8]
        self._moveit2.move_to_configuration(joint_configuration_pre_throw)
        self._moveit2.wait_until_executed()

        # Increase speed
        self._moveit2.max_velocity = 1.0
        self._moveit2.max_acceleration = 1.0

        # Throw itself
        joint_configuration_throw = [0.0, 1.0, 0.0, -1.1, 0.0, 1.9, 0.8]
        self._moveit2.move_to_configuration(joint_configuration_throw)

        # Release object while executing motion
        sleep_duration_s = 1.2
        if rclpy.ok():
            self.create_rate(1 / sleep_duration_s).sleep()
        self._moveit2_gripper.open()
        self._moveit2_gripper.wait_until_executed()
        self._moveit2.wait_until_executed()

        # Return to default configuration
        joint_configuration_default = [
            0.0,
            -0.7853981633974483,
            0.0,
            -2.356194490192345,
            0.0,
            1.5707963267948966,
            0.7853981633974483,
        ]
        self._moveit2.move_to_configuration(joint_configuration_default)
        self._moveit2.wait_until_executed()


def main(args=None):

    rclpy.init(args=args)

    object_thrower = MoveItThrowObject()

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(3)
    executor.add_node(object_thrower)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Wait for everything to setup
    sleep_duration_s = 2.0
    if rclpy.ok():
        object_thrower.create_rate(1 / sleep_duration_s).sleep()

    object_thrower.throw()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
