#!/usr/bin/env python3

import rclpy
from ign_moveit2 import IgnitionMoveIt2Interface


def main(args=None):
    rclpy.init(args=args)

    ign_moveit2 = IgnitionMoveIt2Interface()

    joint_values = [1.5707963,
                    -0.78539816,
                    1.5707963,
                    0.78539816,
                    -1.5707963,
                    1.5707963,
                    0.78539816]
    ign_moveit2.set_joint_goal(joint_values)
    ign_moveit2.plan_trajectory()

    rclpy.spin(ign_moveit2)


if __name__ == "__main__":
    main()
