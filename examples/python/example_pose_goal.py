#!/usr/bin/env python3

from moveit2 import MoveIt2Interface
import rclpy
import threading


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
    quaternion = [1.0, 0.0, 0.0, 0.0]
    moveit2.set_pose_goal(position, quaternion)

    # Plan and execute
    moveit2.plan_kinematic_path()
    moveit2.execute()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
