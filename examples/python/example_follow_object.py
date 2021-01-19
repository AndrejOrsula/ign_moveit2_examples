#!/usr/bin/env python3

from geometry_msgs.msg import Pose
from moveit2 import MoveIt2Interface
from rclpy.node import Node
import rclpy


class ObjectFollower(Node):

    def __init__(self):
        super().__init__("object_follower")

        # Create a subscriber for object pose
        self.previous_object_pose_ = Pose()
        self.object_pose_sub_ = self.create_subscription(Pose, '/model/box/pose',
                                                         self.object_pose_callback, 1)

        # Create MoveIt2 interface node
        self.moveit2_ = MoveIt2Interface()

        # Spin up multi-threaded executor
        self.executor_ = rclpy.executors.MultiThreadedExecutor(2)
        self.executor_.add_node(self)
        self.executor_.add_node(self.moveit2_)
        self.executor_.spin()

    def object_pose_callback(self, pose_msg):
        # Plan trajectory only if object was moved
        if self.previous_object_pose_ != pose_msg:
            self.moveit2_.set_pose_goal([pose_msg.position.x,
                                         pose_msg.position.y,
                                         pose_msg.position.z],
                                        [pose_msg.orientation.x,
                                         pose_msg.orientation.y,
                                         pose_msg.orientation.z,
                                         pose_msg.orientation.w])
            self.moveit2_.plan_kinematic_path()
            self.moveit2_.execute()

            # Update for next callback
            self.previous_object_pose_ = pose_msg


def main(args=None):
    rclpy.init(args=args)

    _object_follower = ObjectFollower()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
