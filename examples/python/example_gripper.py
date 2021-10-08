#!/usr/bin/env python3

from moveit2 import MoveIt2Interface
import rclpy
import threading
import time

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
    moveit2.gripper_open(manual_plan=True)
    moveit2.wait_until_executed()

    time.sleep(2)

    # Close
    moveit2.gripper_close(manual_plan=True)
    moveit2.wait_until_executed()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
