#!/usr/bin/env python3
"""Example that uses MoveIt 2 to follow a target inside Ignition Gazebo"""

from robots import panda

from geometry_msgs.msg import Pose, PoseStamped
from pymoveit2 import MoveIt2
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)
import rclpy


class MoveItFollowTarget(Node):
    def __init__(self):

        super().__init__("ex_follow_target_py")

        # Create callback group that allows execution of callbacks in parallel without restrictions
        self._callback_group = ReentrantCallbackGroup()

        # Create MoveIt2 interface
        self._moveit2 = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            execute_via_moveit=True,
            callback_group=self._callback_group,
        )

        # Create a subscriber for target pose
        self.__previous_target_pose = Pose()
        self.create_subscription(
            msg_type=PoseStamped,
            topic="/target_pose",
            callback=self.target_pose_callback,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=self._callback_group,
        )

        # Setup executor
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(self)

    def target_pose_callback(self, msg: PoseStamped):
        """
        Plan and execute trajectory each time the target pose is changed
        """

        if self.__previous_target_pose != msg.pose:

            # Plan and execute path
            self._moveit2.move_to_pose(
                position=(
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z,
                ),
                quat_xyzw=(
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w,
                ),
            )

            # Update for next callback
            self.__previous_target_pose = msg.pose


def main(args=None):

    rclpy.init(args=args)

    target_follower = MoveItFollowTarget()
    rclpy.spin(target_follower, executor=target_follower.executor)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
