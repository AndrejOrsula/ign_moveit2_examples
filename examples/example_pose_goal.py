#!/usr/bin/env python3

import rclpy
from moveit2 import MoveIt2Interface


def main(args=None):
    rclpy.init(args=args)

    moveit2 = MoveIt2Interface()

    position = [0.5, 0.25, 0.75]
    quaternion = [0.0, 0.0, 0.0, 1.0]
    moveit2.set_pose_goal(position, quaternion)
    moveit2.plan_kinematic_path()
    moveit2.execute()

    rclpy.spin(moveit2)


if __name__ == "__main__":
    main()
