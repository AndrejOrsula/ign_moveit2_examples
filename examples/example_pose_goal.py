#!/usr/bin/env python3

import rclpy
from moveit2 import MoveIt2Interface

import threading
from scipy.spatial.transform import Rotation


def main(args=None):
    rclpy.init(args=args)

    # Initialise MoveIt2
    moveit2 = MoveIt2Interface()

    # Spin MoveIt2 node in the background
    executor = rclpy.executors.MultiThreadedExecutor(1)
    executor.add_node(moveit2)
    thread = threading.Thread(target=executor.spin)
    thread.start()

    # Set pose goal to reach
    position = [0.25, 0.25, 0.5]
    quaternion = Rotation.from_euler('xyz',
                                     [180, 0, 0],
                                     degrees=True).as_quat()
    moveit2.set_pose_goal(position, quaternion)

    # Plan and execute
    moveit2.plan_kinematic_path()
    moveit2.execute()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
