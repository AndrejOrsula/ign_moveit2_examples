#!/usr/bin/env python3

import rclpy
from moveit2 import MoveIt2Interface


def main(args=None):
    rclpy.init(args=args)

    moveit2 = MoveIt2Interface()

    joint_values = [1.5707963,
                    -0.78539816,
                    1.5707963,
                    0.78539816,
                    -1.5707963,
                    1.5707963,
                    0.78539816]
    moveit2.set_joint_goal(joint_values)
    moveit2.plan_kinematic_path()
    moveit2.execute()

    rclpy.spin(moveit2)


if __name__ == "__main__":
    main()
