"""
A simple python interface with MoveIt2 move_group action server for manipulation within Ignition Gazebo. The interface 
with move_group action server is an alternative to moveit_commander, which is not yet ported to ROS 2 (as of Oct 2020).

Note: This module is currently configured for Franka Emika Panda robot.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Quaternion
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint
from moveit_msgs.action import MoveGroup


class IgnitionMoveIt2Interface(Node):

    def __init__(self):
        """
        Setup node as well as its `move_action` action client and JointTrajectory publisher.
        This also initialises request msg to improve performance for repreated requests.
        """
        super().__init__("ign_moveit2_py")
        self.action_client_ = ActionClient(self, MoveGroup, "move_action")
        self.publisher_ = self.create_publisher(JointTrajectory,
                                                "joint_trajectory", 1)
        self.init_robot()
        self.init_request_msg()
        self.get_logger().info("MoveIt2 action client initialised successfuly")

    def init_robot(self):
        """
        Initialise robot groups, links and joints. This would normally get loaded from URDF via `moveit_commander`.
        """
        self.arm_group_name_ = "panda_arm"
        self.arm_joints_ = ["panda_joint1",
                            "panda_joint2",
                            "panda_joint3",
                            "panda_joint4",
                            "panda_joint5",
                            "panda_joint6",
                            "panda_joint7"]
        self.arm_links_ = ["panda_link0",
                           "panda_link1",
                           "panda_link2",
                           "panda_link3",
                           "panda_link4",
                           "panda_link5",
                           "panda_link6",
                           "panda_link7",
                           "panda_link8"]
        self.arm_base_link_ = self.arm_links_[0]
        self.arm_end_effector_ = self.arm_links_[-1]

        # # Gripper
        # TODO: Implement gripper interface
        # self.gripper_group_name_ = "hand"
        # self.gripper_joints_ = ["panda_finger_joint1",
        #                         "panda_finger_joint2"]

    def init_request_msg(self):
        """
        Initialise default requst (goal) for `move_action` action.
        """
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.arm_group_name_
        goal_msg.request.planner_id = ""

        # Workspace based on reachability of robot (panda reach)
        goal_msg.request.workspace_parameters.header.frame_id = self.arm_base_link_
        goal_msg.request.workspace_parameters.min_corner.x = -0.855
        goal_msg.request.workspace_parameters.min_corner.y = -0.855
        goal_msg.request.workspace_parameters.min_corner.z = -0.36
        goal_msg.request.workspace_parameters.max_corner.x = 0.855
        goal_msg.request.workspace_parameters.max_corner.y = 0.855
        goal_msg.request.workspace_parameters.max_corner.z = 1.19

        # Planning attempts
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0

        # Velocity and acceleration
        goal_msg.request.max_velocity_scaling_factor = 0.0
        goal_msg.request.max_acceleration_scaling_factor = 0.0

        # End-effector speed
        goal_msg.request.cartesian_speed_end_effector_link = self.arm_end_effector_
        goal_msg.request.max_cartesian_speed = 0.0

        # We only want to plan trajectory, JointTrajectoryController ignition plugin takes care of execution
        goal_msg.planning_options.plan_only = True

        # Looking around (not supported)
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.look_around_attempts = 0
        goal_msg.planning_options.max_safe_execution_cost = 0.0

        # Replanning if execution fails (not relevant - MoveIt2 is not used for execution here)
        goal_msg.planning_options.replan = False
        goal_msg.planning_options.replan_attempts = 0
        goal_msg.planning_options.replan_delay = 0.0

        # Do not change the start state prior to requesting new trajectories
        goal_msg.request.start_state.is_diff = False

        # Create a single contraint for goals, which is updated when setting them
        goal_msg.request.goal_constraints = [Constraints()]

        # Save message template to internals
        self.goal_msg_ = goal_msg

    def clear_goal(self):
        """
        Clear all goal constraints that were previously set.
        Note that this function is called after each successful `plan_trajectory()`.
        """
        self.goal_msg_.request.goal_constraints = [Constraints()]

    def create_new_goal_constraint(self):
        """
        Create a new set of goal contraints that will be set together with the request. Each subsequent setting of goals
        with `set_joint_goal()`, `set_pose_goal()` and others will be added under this newly created set of contraints.
        """
        self.goal_msg_.request.goal_constraints.append(Constraints())

    def set_joint_goal(self, joint_positions, tolerance=0.001, weight=1.0, joint_names=None):
        """
        Set goal position in joint space. With `joint_names` specified, `joint_positions` can be defined for specific joints.
        Otherwise, first `n` joints defined in `init_robot()` will be used, where `n` is the length of `joint_positions`.
        """
        if joint_names == None:
            joint_names = self.arm_joints_

        for i in range(len(joint_positions)):
            goal = JointConstraint()
            goal.joint_name = joint_names[i]
            goal.position = joint_positions[i]
            goal.tolerance_above = tolerance
            goal.tolerance_below = tolerance
            goal.weight = weight

            self.goal_msg_.request.goal_constraints[-1].joint_constraints.append(
                goal)

    def set_position_goal(self, position, tolerance=0.001, weight=1.0, frame=None):
        """
        Set goal position of `frame` in Cartesian space. Defaults to the end-effector `frame`.
        """
        if frame == None:
            frame = self.arm_base_link_

        goal = PositionConstraint()
        goal.header.frame_id = frame
        goal.link_name = self.arm_end_effector_
        goal.constraint_region.primitive_poses.append(Pose())
        goal.constraint_region.primitive_poses[0].position.x = position[0]
        goal.constraint_region.primitive_poses[0].position.y = position[1]
        goal.constraint_region.primitive_poses[0].position.z = position[2]

        # Goal is defined as a sphere with radius equal to tolerance
        goal.constraint_region.primitives.append(SolidPrimitive())
        goal.constraint_region.primitives[0].type = 2
        goal.constraint_region.primitives[0].dimensions = [tolerance]
        goal.weight = weight

        self.goal_msg_.request.goal_constraints[-1].position_constraints.append(
            goal)

    def set_orientation_goal(self, quaternion, tolerance=0.001, weight=1.0, frame=None):
        """
        Set goal orientation of `frame`. Defaults to the end-effector `frame`.
        """
        if frame == None:
            frame = self.arm_base_link_

        goal = OrientationConstraint()
        goal.header.frame_id = frame
        goal.link_name = self.arm_end_effector_
        goal.orientation.x = quaternion[0]
        goal.orientation.y = quaternion[1]
        goal.orientation.z = quaternion[2]
        goal.orientation.w = quaternion[3]
        goal.absolute_x_axis_tolerance = tolerance
        goal.absolute_y_axis_tolerance = tolerance
        goal.absolute_z_axis_tolerance = tolerance
        goal.weight = weight

        self.goal_msg_.request.goal_constraints[-1].orientation_constraints.append(
            goal)

    def set_pose_goal(self, position, quaternion,
                      tolerance_position=0.001, tolerance_orientation=0.001,
                      weight_position=1.0, weight_orientation=1.0,
                      frame=None):
        """
        Set goal pose. This is direct combination of `set_position_goal()` and `set_orientation_goal()`.
        """
        self.set_position_goal(
            position, tolerance_position, weight_position, frame)
        self.set_orientation_goal(
            quaternion, tolerance_orientation, weight_orientation, frame)

    def plan_trajectory(self):
        """
        Plan trajectory with previously set goals. The plan is automatically publised on success with `pub_joint_trajectory()`.
        TODO: Return response/feedback in a nice way.
        """
        # Stamp message with current time
        clock_time_now_msg = self._clock.now().to_msg()
        self.goal_msg_.request.workspace_parameters.header.stamp = clock_time_now_msg
        for contraints in self.goal_msg_.request.goal_constraints:
            for position_constraint in contraints.position_constraints:
                position_constraint.header.stamp = clock_time_now_msg
            for orientation_constraint in contraints.orientation_constraints:
                orientation_constraint.header.stamp = clock_time_now_msg

        self.get_logger().info("Waiting for action server...")
        self.action_client_.wait_for_server()

        self.get_logger().info("Sending plan request...")
        self._send_goal_future = self.action_client_.send_goal_async(self.goal_msg_,
                                                                     feedback_callback=self.action_feedback_callback)

        # Register response callback
        self._send_goal_future.add_done_callback(self.action_response_callback)

    def action_feedback_callback(self, feedback):
        """
        Feedback callback from `move_action` server that is logged.
        """
        self.get_logger().info("Received feedback state: {0}"
                               .format(feedback.feedback.state))

    def action_response_callback(self, future):
        """
        Response callback from `move_action` server that registers result callback.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Planning rejected")
            return

        # Register result callback
        self.get_logger().info("Planning accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.action_result_callback)

    def action_result_callback(self, future):
        """
        Result callback from `move_action` server that publishes JointTrajectory on success with `pub_joint_trajectory()`.
        """
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Planning successful, publishing JointTrajectory for execution...")
            self.pub_joint_trajectory(
                future.result().result.planned_trajectory)
        else:
            self.get_logger().error(
                "Planning failed with status: {0}".format(status))

    def pub_joint_trajectory(self, moveit_trajectory):
        """
        Publish JointTrajectory, such that it can be executed by JointTrajectoryController.
        This function also clears all previously set goals with `clear_goal()`.
        """
        self.publisher_.publish(moveit_trajectory.joint_trajectory)
        self.clear_goal()
