#!/usr/bin/env python3

import rclpy
from ign_moveit2 import IgnitionMoveIt2Interface


def main(args=None):
    rclpy.init(args=args)

    ign_moveit2 = IgnitionMoveIt2Interface()

    position = [0.5, 0.25, 0.75]
    quaternion = [0.0, 0.0, 0.0, 1.0]
    ign_moveit2.set_pose_goal(position, quaternion)
    ign_moveit2.plan_trajectory()

    rclpy.spin(ign_moveit2)


if __name__ == "__main__":
    main()
