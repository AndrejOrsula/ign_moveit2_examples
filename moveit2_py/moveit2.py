"""
A simple python interface with MoveIt2 services (and actions). This is an alternative to moveit_commander,
which is not yet ported to ROS 2 (as of Oct 2020).

Note: This module is currently configured for Franka Emika Panda robot.
Note: There is no Ignition-specific code in this module (this repo is just a convinient place for it).
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Float32
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, Quaternion
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, PositionIKRequest, RobotTrajectory
from moveit_msgs.srv import GetPositionIK, GetPositionFK, GetMotionPlan, GetCartesianPath
from moveit_msgs.action import MoveGroup
from action_msgs.msg import GoalStatus


class MoveIt2Interface(Node):

    def __init__(self):
        super().__init__("ign_moveit2_py")
        self.init_robot()
        self.init_compute_fk()
        self.init_compute_ik()
        self.init_plan_kinematic_path()
        self.init_plan_cartesian_path()
        self.init_gripper()
        self.get_logger().info("ign_moveit2_py initialised successfuly")

    def init_robot(self):
        """
        Initialise robot groups, links and joints. This would normally get loaded from URDF via `moveit_commander`.
        This also initialises subscriber to joint states and publisher to joint trajectories.
        """
        self.robot_group_name_ = "panda_arm_hand"
        # Arm
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
        # Gripper
        self.gripper_group_name_ = "hand"
        self.gripper_joints_ = ["panda_finger_joint1",
                                "panda_finger_joint2"]
        self.gripper_links_ = ["panda_leftfinger",
                               "panda_rightfinger"]
        self.gripper_max_speed_ = 0.2

        # Publisher of trajectories
        self.joint_trajectory_pub_ = self.create_publisher(JointTrajectory,
                                                           "joint_trajectory", 1)

        # Subscriber of current joint states
        self.joint_state_ = JointState()
        self.joint_state_mutex_ = threading.Lock()
        self.joint_state_sub_ = self.create_subscription(JointState,
                                                         "joint_states",
                                                         self.joint_state_callback, 1)

        # Subscriber of joint trajectory progress
        self.joint_progress_ = 1.0
        self.joint_progress_cond_ = threading.Condition()
        self.joint_progress_sub_ = self.create_subscription(Float32,
                                                            "joint_trajectory_progress",
                                                            self.joint_progress_callback, 1)

    def joint_state_callback(self, msg):
        """
        Callback for getting current joint states.
        """
        self.joint_state_mutex_.acquire()
        self.joint_state_ = msg
        self.joint_state_mutex_.release()

    def get_joint_state(self) -> JointState:
        """
        Get current joint states.
        """
        self.joint_state_mutex_.acquire()
        joint_state = self.joint_state_
        self.joint_state_mutex_.release()
        return joint_state

    def joint_progress_callback(self, msg):
        """
        Callback for getting joint trajectory progress.
        """
        with self.joint_progress_cond_:
            self.joint_progress_ = msg.data
            self.joint_progress_cond_.notify_all()

    def wait_until_executed(self):
        """
        Function that halts execution on the current thread until trajectory is executed.
        """
        with self.joint_progress_cond_:
            while not self.joint_progress_ == 1.0:
                self.joint_progress_cond_.wait(timeout=0.5)

    def pub_trajectory(self, trajectory):
        """
        Publish trajectory such that it can be executed, e.g. by `JointTrajectoryController` Ignition plugin.
        """
        if isinstance(trajectory, JointTrajectory):
            self.joint_trajectory_pub_.publish(trajectory)
        elif isinstance(trajectory, RobotTrajectory):
            self.joint_trajectory_pub_.publish(trajectory.joint_trajectory)
        else:
            raise Exception("Invalid type passed into pub_trajectory()")

    def execute(self, joint_trajectory=None) -> bool:
        """
        Execute last planned motion plan, or the `joint_trajectory` specified as argument.
        """

        if joint_trajectory == None:
            plan = self.motion_plan_.joint_trajectory
        else:
            plan = joint_trajectory

        # Make sure there is a plan to follow
        if not plan.points:
            self.get_logger().warn(
                "Cannot execute motion plan because it does not contain any trajectory points")
            return False

        # Reset joint progress
        self.joint_progress_ = 0.0

        self.pub_trajectory(plan)
        return True

    def move_to_joint_state(self, joint_state, set_position=True, set_velocity=True, set_effort=True):
        """
        Set joint target on all joints defined in `joint_state`. This function does NOT plan a smooth
        trajectory and only publishes joint_state as the next goal that should be reached immediately.
        """
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = joint_state.name

        point = JointTrajectoryPoint()
        if set_position:
            point.positions = joint_state.position
        if set_velocity:
            point.velocities = joint_state.velocity
        if set_effort:
            point.effort = joint_state.effort
        joint_trajectory.points.append(point)

        self.pub_trajectory(joint_trajectory)

    # compute_fk
    def init_compute_fk(self):
        """
        Initialise `compute_fk` service.
        """
        self.compute_fk_client_ = self.create_client(GetPositionFK,
                                                     "compute_fk")
        while not self.compute_fk_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Service [compute_fk] not currently available, waiting...")

        self.fk_request_ = GetPositionFK.Request()
        self.fk_request_.header.frame_id = self.arm_base_link_
        # self.fk_request_.header.stamp = "Set during request"
        # self.fk_request_.fk_link_names = "Set during request"
        # self.fk_request_.robot_state.joint_state = "Set during request"
        # self.fk_request_.robot_state.multi_dof_joint_state = "Ignored"
        # self.fk_request_.robot_state.attached_collision_objects = "Ignored"
        self.fk_request_.robot_state.is_diff = False

    def compute_fk(self, fk_link_names=None, joint_state=None) -> GetPositionFK.Response:
        """
        Call `compute_fk` service.
        """
        if fk_link_names == None:
            self.fk_request_.fk_link_names = [self.arm_end_effector_]
        else:
            self.fk_request_.fk_link_names = fk_link_names

        if joint_state == None:
            self.fk_request_.robot_state.joint_state = self.get_joint_state()
        else:
            self.fk_request_.robot_state.joint_state = joint_state

        self.fk_request_.header.stamp = self._clock.now().to_msg()

        self.compute_fk_client_.wait_for_service()
        return self.compute_fk_client_.call(self.fk_request_)

    # compute_ik
    def init_compute_ik(self):
        """
        Initialise `compute_ik` service.
        """
        # Service client for IK
        self.compute_ik_client_ = self.create_client(GetPositionIK,
                                                     "compute_ik")
        while not self.compute_ik_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Service [compute_ik] not currently available, waiting...")

        self.ik_request_ = GetPositionIK.Request()
        self.ik_request_.ik_request.group_name = self.arm_group_name_
        # self.ik_request_.ik_request.robot_state.joint_state = "Set during request"
        # self.ik_request_.ik_request.robot_state.multi_dof_joint_state = "Ignored"
        # self.ik_request_.ik_request.robot_state.attached_collision_objects = "Ignored"
        self.ik_request_.ik_request.robot_state.is_diff = False
        # self.ik_request_.ik_request.constraints = "Set during request OR Ignored"
        self.ik_request_.ik_request.avoid_collisions = True
        # self.ik_request_.ik_request.ik_link_name = "Ignored"
        self.ik_request_.ik_request.pose_stamped.header.frame_id = self.arm_base_link_
        # self.ik_request_.ik_request.pose_stamped.header.stamp = "Set during request"
        # self.ik_request_.ik_request.pose_stamped.pose = "Set during request"
        # self.ik_request_.ik_request.ik_link_names = "Ignored"
        # self.ik_request_.ik_request.pose_stamped_vector = "Ignored"
        # self.ik_request_.ik_request.timeout.sec = "Ignored"
        # self.ik_request_.ik_request.timeout.nanosec = "Ignored"

    def compute_ik(self, pose, start_joint_state=None, constrains=None) -> GetPositionIK.Response:
        """
        Call `compute_ik` service.
        """
        if start_joint_state == None:
            self.ik_request_.ik_request.robot_state.joint_state = self.get_joint_state()
        else:
            self.ik_request_.ik_request.robot_state.joint_state = start_joint_state

        if constrains != None:
            self.ik_request_.ik_request.constraints = constrains

        self.ik_request_.ik_request.pose_stamped.pose = pose

        self.ik_request_.ik_request.pose_stamped.header.stamp = self._clock.now().to_msg()

        self.compute_ik_client_.wait_for_service()
        return self.compute_ik_client_.call(self.ik_request_)

    # plan_kinematic_path
    def init_plan_kinematic_path(self):
        """
        Initialise `plan_kinematic_path` service.
        """
        # Service client for IK
        self.plan_kinematic_path_client_ = self.create_client(GetMotionPlan,
                                                              "plan_kinematic_path")
        while not self.plan_kinematic_path_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Service [plan_kinematic_path] not currently available, waiting...")

        self.kinematic_path_request_ = GetMotionPlan.Request()
        self.kinematic_path_request_.motion_plan_request.workspace_parameters.header.frame_id = self.arm_base_link_
        # self.kinematic_path_request_.motion_plan_request.workspace_parameters.header.stamp = "Set during request"
        self.kinematic_path_request_.motion_plan_request.workspace_parameters.min_corner.x = -0.855
        self.kinematic_path_request_.motion_plan_request.workspace_parameters.min_corner.y = -0.855
        self.kinematic_path_request_.motion_plan_request.workspace_parameters.min_corner.z = -0.36
        self.kinematic_path_request_.motion_plan_request.workspace_parameters.max_corner.x = 0.855
        self.kinematic_path_request_.motion_plan_request.workspace_parameters.max_corner.y = 0.855
        self.kinematic_path_request_.motion_plan_request.workspace_parameters.max_corner.z = 1.19
        # self.kinematic_path_request_.motion_plan_request.start_state = "Ignored"
        self.kinematic_path_request_.motion_plan_request.goal_constraints = [
            Constraints()]
        # self.kinematic_path_request_.motion_plan_request.path_constraints = "Ignored"
        # self.kinematic_path_request_.motion_plan_request.trajectory_constraints = "Ignored"
        # self.kinematic_path_request_.motion_plan_request.reference_trajectories = "Ignored"
        # self.kinematic_path_request_.motion_plan_request.planner_id = "Ignored"
        self.kinematic_path_request_.motion_plan_request.group_name = self.arm_group_name_
        # TODO: Make configurable
        self.kinematic_path_request_.motion_plan_request.num_planning_attempts = 10
        # TODO: Make configurable
        self.kinematic_path_request_.motion_plan_request.allowed_planning_time = 5.0
        self.kinematic_path_request_.motion_plan_request.max_velocity_scaling_factor = 0.0
        self.kinematic_path_request_.motion_plan_request.max_acceleration_scaling_factor = 0.0
        self.kinematic_path_request_.motion_plan_request.cartesian_speed_end_effector_link = self.arm_end_effector_
        self.kinematic_path_request_.motion_plan_request.max_cartesian_speed = 0.0

    def plan_kinematic_path(self) -> GetMotionPlan.Response:
        """
        Call `plan_kinematic_path` service, with goal set using either `set_joint_goal()`, `set_position_goal()`,
        `set_orientation_goal()` or `set_pose_goal()`.
        This function automatically calls `plan_kinematic_path()`.
        """
        self.kinematic_path_request_.motion_plan_request.workspace_parameters.header.stamp = self._clock.now().to_msg()

        # Stamp message with current time
        clock_time_now_msg = self._clock.now().to_msg()
        self.kinematic_path_request_.motion_plan_request.workspace_parameters.header.stamp = clock_time_now_msg
        for contraints in self.kinematic_path_request_.motion_plan_request.goal_constraints:
            for position_constraint in contraints.position_constraints:
                position_constraint.header.stamp = clock_time_now_msg
            for orientation_constraint in contraints.orientation_constraints:
                orientation_constraint.header.stamp = clock_time_now_msg

        self.plan_kinematic_path_client_.wait_for_service()
        response = self.plan_kinematic_path_client_.call(
            self.kinematic_path_request_)

        self.clear_goal_constraints()
        self.motion_plan_ = response.motion_plan_response.trajectory

        return response

    def clear_goal_constraints(self):
        """
        Clear all goal constraints that were previously set.
        Note that this function is called automatically after each `plan_kinematic_path()`.
        """
        self.kinematic_path_request_.motion_plan_request.goal_constraints = [
            Constraints()]

    def create_new_goal_constraint(self):
        """
        Create a new set of goal contraints that will be set together with the request. Each subsequent setting of goals
        with `set_joint_goal()`, `set_pose_goal()` and others will be added under this newly created set of contraints.
        """
        self.kinematic_path_request_.motion_plan_request.goal_constraints.append(
            Constraints())

    def set_joint_goal(self, joint_positions, tolerance=0.001, weight=1.0, joint_names=None):
        """
        Set goal position in joint space. With `joint_names` specified, `joint_positions` can be defined for specific joints.
        Otherwise, first `n` joints defined in `init_robot()` will be used, where `n` is the length of `joint_positions`.
        """
        if joint_names == None:
            joint_names = self.arm_joints_

        for i in range(len(joint_positions)):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_names[i]
            joint_constraint.position = joint_positions[i]
            joint_constraint.tolerance_above = tolerance
            joint_constraint.tolerance_below = tolerance
            joint_constraint.weight = weight

            self.kinematic_path_request_.motion_plan_request.goal_constraints[-1].joint_constraints.append(
                joint_constraint)

    def set_position_goal(self, position, tolerance=0.001, weight=1.0, frame=None):
        """
        Set goal position of `frame` in Cartesian space. Defaults to the end-effector `frame`.
        """
        if frame == None:
            frame = self.arm_base_link_

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = frame
        position_constraint.link_name = self.arm_end_effector_
        position_constraint.constraint_region.primitive_poses.append(Pose())
        position_constraint.constraint_region.primitive_poses[0].position.x = position[0]
        position_constraint.constraint_region.primitive_poses[0].position.y = position[1]
        position_constraint.constraint_region.primitive_poses[0].position.z = position[2]

        # Goal is defined as a sphere with radius equal to tolerance
        position_constraint.constraint_region.primitives.append(
            SolidPrimitive())
        position_constraint.constraint_region.primitives[0].type = 2
        position_constraint.constraint_region.primitives[0].dimensions = [
            tolerance]
        position_constraint.weight = weight

        self.kinematic_path_request_.motion_plan_request.goal_constraints[-1].position_constraints.append(
            position_constraint)

    def set_orientation_goal(self, quaternion, tolerance=0.001, weight=1.0, frame=None):
        """
        Set goal orientation of `frame`. Defaults to the end-effector `frame`.
        """
        if frame == None:
            frame = self.arm_end_effector_

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.arm_base_link_
        orientation_constraint.link_name = frame
        orientation_constraint.orientation.x = quaternion[0]
        orientation_constraint.orientation.y = quaternion[1]
        orientation_constraint.orientation.z = quaternion[2]
        orientation_constraint.orientation.w = quaternion[3]
        orientation_constraint.absolute_x_axis_tolerance = tolerance
        orientation_constraint.absolute_y_axis_tolerance = tolerance
        orientation_constraint.absolute_z_axis_tolerance = tolerance
        orientation_constraint.weight = weight

        self.kinematic_path_request_.motion_plan_request.goal_constraints[-1].orientation_constraints.append(
            orientation_constraint)

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

    # plan_cartesian_path
    def init_plan_cartesian_path(self):
        """
        Initialise `compute_cartesian_path` service.
        """
        # TODO
        pass

    def plan_cartesian_path(self, pose, start_joint_state=None, constrains=None) -> GetCartesianPath.Response:
        """
        Call `compute_cartesian_path` service.
        """
        # TODO
        pass

    def init_gripper(self):
        """
        Initialise `gripper` service.
        """
        # Service client for IK
        self.plan_gripper_path_client_ = self.create_client(GetMotionPlan,
                                                            "plan_kinematic_path")
        while not self.plan_gripper_path_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Service [plan_kinematic_path] not currently available, waiting...")

        self.gripper_path_request_ = GetMotionPlan.Request()
        self.gripper_path_request_.motion_plan_request.workspace_parameters.header.frame_id = self.arm_base_link_
        # self.gripper_path_request_.motion_plan_request.workspace_parameters.header.stamp = "Set during request"
        self.gripper_path_request_.motion_plan_request.workspace_parameters.min_corner.x = -0.855
        self.gripper_path_request_.motion_plan_request.workspace_parameters.min_corner.y = -0.855
        self.gripper_path_request_.motion_plan_request.workspace_parameters.min_corner.z = -0.36
        self.gripper_path_request_.motion_plan_request.workspace_parameters.max_corner.x = 0.855
        self.gripper_path_request_.motion_plan_request.workspace_parameters.max_corner.y = 0.855
        self.gripper_path_request_.motion_plan_request.workspace_parameters.max_corner.z = 1.19
        # self.gripper_path_request_.motion_plan_request.start_state = "Ignored"
        self.gripper_path_request_.motion_plan_request.goal_constraints = [
            Constraints()]
        self.gripper_path_request_.motion_plan_request.goal_constraints[0].joint_constraints.append(
            JointConstraint())
        self.gripper_path_request_.motion_plan_request.goal_constraints[0].joint_constraints[0].joint_name = self.gripper_joints_[
            0]
        # self.gripper_path_request_.motion_plan_request.goal_constraints[0].joint_constraints[0].position = "Set during request"
        # self.gripper_path_request_.motion_plan_request.goal_constraints[0].joint_constraints[0].tolerance_above = "Ignored"
        # self.gripper_path_request_.motion_plan_request.goal_constraints[0].joint_constraints[0].tolerance_below = "Ignored"
        self.gripper_path_request_.motion_plan_request.goal_constraints[
            0].joint_constraints[0].weight = 1.0
        # self.gripper_path_request_.motion_plan_request.path_constraints = "Ignored"
        # self.gripper_path_request_.motion_plan_request.trajectory_constraints = "Ignored"
        # self.gripper_path_request_.motion_plan_request.reference_trajectories = "Ignored"
        # self.gripper_path_request_.motion_plan_request.planner_id = "Ignored"
        self.gripper_path_request_.motion_plan_request.group_name = self.gripper_group_name_
        self.gripper_path_request_.motion_plan_request.num_planning_attempts = 1
        self.gripper_path_request_.motion_plan_request.allowed_planning_time = 0.1
        # self.gripper_path_request_.motion_plan_request.max_velocity_scaling_factor = "Set during request"
        self.gripper_path_request_.motion_plan_request.max_acceleration_scaling_factor = 0.0
        # self.gripper_path_request_.motion_plan_request.cartesian_speed_end_effector_link = "Ignored"
        # self.gripper_path_request_.motion_plan_request.max_cartesian_speed = "Ignored"

    def gripper_plan_path(self, width, speed) -> JointTrajectory:
        self.gripper_path_request_.motion_plan_request.max_velocity_scaling_factor = speed / \
            self.gripper_max_speed_
        self.gripper_path_request_.motion_plan_request.goal_constraints[
            0].joint_constraints[0].position = width/2

        self.plan_gripper_path_client_.wait_for_service()
        response = self.plan_gripper_path_client_.call(
            self.gripper_path_request_)

        joint_trajectory = response.motion_plan_response.trajectory.joint_trajectory

        # Mirror motion on the second finger (might be more efficient than double planning)
        joint_trajectory.joint_names.append(self.gripper_joints_[1])
        for i in range(len(joint_trajectory.points)):
            if joint_trajectory.points[i].positions:
                joint_trajectory.points[i].positions.append(
                    joint_trajectory.points[i].positions[0])
            if joint_trajectory.points[i].velocities:
                joint_trajectory.points[i].velocities.append(
                    joint_trajectory.points[i].velocities[0])
            if joint_trajectory.points[i].accelerations:
                joint_trajectory.points[i].accelerations.append(
                    joint_trajectory.points[i].accelerations[0])
            if joint_trajectory.points[i].effort:
                joint_trajectory.points[i].effort.append(
                    joint_trajectory.points[i].effort[0])

        return joint_trajectory

    def gripper_close(self, width=0.0, speed=0.2, force=20.0) -> bool:
        joint_trajectory = self.gripper_plan_path(width, speed)

        if not joint_trajectory.points:
            return False

        # Set the desired force on the last point
        joint_trajectory.points[-1].effort = [-force, -force]

        return self.execute(joint_trajectory)

    def gripper_open(self, width=0.08, speed=0.2) -> bool:
        joint_trajectory = self.gripper_plan_path(width, speed)

        if not joint_trajectory.points:
            return False

        # Make sure no force is applied anymore
        joint_trajectory.points[0].effort = [0.0, 0.0]

        return self.execute(joint_trajectory)

    # # move_action
    # # Note: Use `plan_kinematic_path()` or `plan_cartesian_path()` together with `execute()` instead
    # def init_move_action(self):
    #     """
    #     Initialise `move_action` action.
    #     """
    #     # Service client for path planning
    #     self.move_action_client_ = ActionClient(self, MoveGroup,
    #                                             "move_action")
    #     while not self.move_action_client_.wait_for_server(timeout_sec=1.0):
    #         self.get_logger().info("Action [move_action] not currently available, waiting...")

    #     self.move_action_goal_ = MoveGroup.Goal()

    #     # Cloned from `plan_kinematic_path` service
    #     self.move_action_goal_.request = self.kinematic_path_request_

    #     # self.move_action_goal_.planning_options.planning_scene_diff = "Ignored"
    #     self.move_action_goal_.planning_options.plan_only = True
    #     # self.move_action_goal_.planning_options.look_around = "Ignored"
    #     # self.move_action_goal_.planning_options.look_around_attempts = "Ignored"
    #     # self.move_action_goal_.planning_options.max_safe_execution_cost = "Ignored"
    #     # self.move_action_goal_.planning_options.replan = "Ignored"
    #     # self.move_action_goal_.planning_options.replan_attempts = "Ignored"
    #     # self.move_action_goal_.planning_options.replan_delay = "Ignored"

    # def move_action(self):
    #     """
    #     Call `move_action` action.
    #     Note: Please use `plan_kinematic_path()` or `plan_cartesian_path()` together with `execute()` instead
    #     """
    #     # Cloned from `plan_kinematic_path` service
    #     self.move_action_goal_.request = self.kinematic_path_request_

    #     # Stamp message with current time
    #     clock_time_now_msg = self._clock.now().to_msg()
    #     self.move_action_goal_.request.workspace_parameters.header.stamp = clock_time_now_msg
    #     for contraints in self.move_action_goal_.request.goal_constraints:
    #         for position_constraint in contraints.position_constraints:
    #             position_constraint.header.stamp = clock_time_now_msg
    #         for orientation_constraint in contraints.orientation_constraints:
    #             orientation_constraint.header.stamp = clock_time_now_msg

    #     self.move_action_client_.wait_for_server()
    #     self.move_action_goal_future_ = self.move_action_client_.send_goal_async(self.move_action_goal_,
    #                                                                              feedback_callback=self.move_action_feedback_callback)
    #     self.move_action_goal_future_.add_done_callback(
    #         self.move_action_response_callback)

    #     self.clear_goal_constraints()

    # def move_action_response_callback(self, future):
    #     """
    #     Response callback from `move_action` server that registers result callback.
    #     """
    #     goal_handle = future.result()
    #     if not goal_handle.accepted:
    #         self.get_logger().error("Planning rejected")
    #         return

    #     # Register result callback
    #     self.get_logger().info("Planning accepted")
    #     self.get_result_future_ = goal_handle.get_result_async()
    #     self.get_result_future_.add_done_callback(
    #         self.move_action_result_callback)

    # def move_action_feedback_callback(self, feedback):
    #     """
    #     Feedback callback from `move_action` server that is logged.
    #     """
    #     self.get_logger().info("Received feedback state: {0}"
    #                            .format(feedback.feedback.state))

    # def move_action_result_callback(self, future):
    #     """
    #     Result callback from `move_action` server that publishes JointTrajectory on success with `pub_trajectory()`.
    #     This function automatically clears all previously set goals with `clear_goal_constraints()`.
    #     """
    #     status = future.result().status
    #     if status == GoalStatus.STATUS_SUCCEEDED:
    #         self.get_logger().info("Planning successful, publishing JointTrajectory for execution...")
    #         self.motion_plan_ = future.result().result.planned_trajectory
    #         self.execute()
    #     else:
    #         self.get_logger().error(
    #             "Planning failed with status: {0}".format(status))
