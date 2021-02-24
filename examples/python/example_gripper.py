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

    # Open
    moveit2.gripper_open(width=0.08, speed=0.2)
    moveit2.wait_until_executed()

    # Close
    moveit2.gripper_close(width=0.0, speed=0.2, force=20.0, force_start=0.75)
    moveit2.wait_until_executed()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
