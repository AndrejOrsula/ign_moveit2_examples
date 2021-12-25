#!/usr/bin/env python3
"""Example that uses MoveIt 2 to follow a target inside Ignition Gazebo"""

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from pymoveit2 import MoveIt2
from pymoveit2.robots import panda
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile


class MoveItFollowTarget(Node):
    def __init__(self):

        super().__init__("ex_follow_target_py")

        # Create callback group that allows execution of callbacks in parallel without restrictions
        self._callback_group = ReentrantCallbackGroup()

        # Create MoveIt 2 interface
        self._moveit2 = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_ARM,
            execute_via_moveit=True,
            callback_group=self._callback_group,
        )
        # Use upper joint velocity and acceleration limits
        self._moveit2.max_velocity = 1.0
        self._moveit2.max_acceleration = 1.0

        # Create a subscriber for target pose
        self.__previous_target_pose = Pose()
        self.create_subscription(
            msg_type=PoseStamped,
            topic="/target_pose",
            callback=self.target_pose_callback,
            qos_profile=QoSProfile(depth=1),
            callback_group=self._callback_group,
        )

        self.get_logger().info("Initialization successful.")

    def target_pose_callback(self, msg: PoseStamped):
        """
        Plan and execute trajectory each time the target pose is changed
        """

        # Return if target pose is unchanged
        if msg.pose == self.__previous_target_pose:
            return

        self.get_logger().info("Target pose has changed. Planning and executing...")

        # Plan and execute motion
        self._moveit2.move_to_pose(
            position=msg.pose.position,
            quat_xyzw=msg.pose.orientation,
        )

        # Update for next callback
        self.__previous_target_pose = msg.pose


def main(args=None):

    rclpy.init(args=args)

    target_follower = MoveItFollowTarget()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(target_follower)
    executor.spin()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
