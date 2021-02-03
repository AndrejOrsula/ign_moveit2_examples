#!/usr/bin/env python3

from geometry_msgs.msg import Pose
from moveit2 import MoveIt2Interface
from rclpy.node import Node
import rclpy
import time


class Thrower(Node):

    def __init__(self):
        super().__init__("thrower")
        # Create a subscriber for object pose
        self._object_pose_sub = self.create_subscription(Pose, '/model/throwing_object/pose',
                                                         self.object_pose_callback, 1)

        # Create MoveIt2 interface node
        self._moveit2 = MoveIt2Interface()

        # Create multi-threaded executor
        self._executor = rclpy.executors.MultiThreadedExecutor(2)
        self._executor.add_node(self)
        self._executor.add_node(self._moveit2)

        # Wait a couple of seconds until Ignition is ready and spin up the executor
        time.sleep(2)
        self._executor.spin()

    def object_pose_callback(self, pose_msg):
        self.throw(pose_msg.position)
        self.destroy_subscription(self._object_pose_sub)
        rclpy.shutdown()
        exit(0)

    def throw(self, object_position):
        # Open gripper
        self._moveit2.gripper_open()
        self._moveit2.wait_until_executed()

        # Move above object
        position = [object_position.x,
                    object_position.y, object_position.z + 0.1]
        quaternion = [1.0, 0.0, 0.0, 0.0]
        self._moveit2.set_pose_goal(position, quaternion)
        self._moveit2.plan_kinematic_path()
        self._moveit2.execute()
        self._moveit2.wait_until_executed()

        self._moveit2.set_max_velocity(0.5)
        self._moveit2.set_max_acceleration(0.5)

        # Move to grasp position
        position = [object_position.x,
                    object_position.y, object_position.z]
        quaternion = [1.0, 0.0, 0.0, 0.0]
        self._moveit2.set_pose_goal(position, quaternion)
        self._moveit2.plan_kinematic_path()
        self._moveit2.execute()
        self._moveit2.wait_until_executed()

        # Close gripper
        self._moveit2.gripper_close(width=0.05, speed=0.2, force=20.0)
        self._moveit2.wait_until_executed()

        # Move above object again
        position = [object_position.x,
                    object_position.y, object_position.z + 0.1]
        quaternion = [1.0, 0.0, 0.0, 0.0]
        self._moveit2.set_pose_goal(position, quaternion)
        self._moveit2.plan_kinematic_path()
        self._moveit2.execute()
        self._moveit2.wait_until_executed()

        # Move to pre-throw configuration
        joint_positions = [0.0,
                           -1.75,
                           0.0,
                           -0.1,
                           0.0,
                           3.6,
                           0.8]
        self._moveit2.set_joint_goal(joint_positions)
        self._moveit2.plan_kinematic_path()
        self._moveit2.execute()
        self._moveit2.wait_until_executed()

        # Throw
        self._moveit2.set_max_velocity(1.0)
        self._moveit2.set_max_acceleration(1.0)

        # Arm trajectory
        joint_positions = [0.0,
                           1.0,
                           0.0,
                           -1.1,
                           0.0,
                           1.9,
                           0.8]
        self._moveit2.set_joint_goal(joint_positions)
        trajectory = self._moveit2.plan_kinematic_path(
        ).motion_plan_response.trajectory.joint_trajectory

        # Hand opening trajectory
        hand_trajectory = self._moveit2.gripper_plan_path(0.08, 0.2)

        # Merge hand opening into arm trajectory, such that it is timed for release (at 50%)
        release_index = round(0.5*len(trajectory.points))
        for finger_joint in hand_trajectory.joint_names:
            trajectory.joint_names.append(finger_joint)
        while len(trajectory.points[release_index].effort) < 9:
            trajectory.points[release_index].effort.append(0.0)
        for finger_index in range(2):
            trajectory.points[release_index].positions.append(
                hand_trajectory.points[-1].positions[finger_index])
            trajectory.points[release_index].velocities.append(
                hand_trajectory.points[-1].velocities[finger_index])
            trajectory.points[release_index].accelerations.append(
                hand_trajectory.points[-1].accelerations[finger_index])

        self._moveit2.execute(trajectory)
        self._moveit2.wait_until_executed()

        # Move to default position
        joint_positions = [0.0,
                           0.0,
                           0.0,
                           -1.57,
                           0.0,
                           1.57,
                           0.79]
        self._moveit2.set_joint_goal(joint_positions)
        self._moveit2.plan_kinematic_path()
        self._moveit2.execute()
        self._moveit2.wait_until_executed()


def main(args=None):
    rclpy.init(args=args)

    _thrower = Thrower()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
