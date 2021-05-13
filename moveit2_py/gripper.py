"""
TODO: docs for gripper interface
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
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint
from moveit_msgs.msg import PositionIKRequest, RobotTrajectory
from moveit_msgs.srv import GetPositionIK, GetPositionFK, GetMotionPlan, GetCartesianPath
from moveit_msgs.action import MoveGroup
from action_msgs.msg import GoalStatus


# TODO: Refactor and separate Gripper interface

class GripperInterface(Node):

    def __init__(self, separate_gripper_controller=False):
        super().__init__("ign_moveit2_py")

        self.init_robot(separate_gripper_controller=separate_gripper_controller)
        self.init_gripper()
        self.get_logger().info("ign_moveit2_py initialised successfuly")

    def init_robot(self, separate_gripper_controller=False):
        """
        Initialise robot groups, links and joints. This would normally get loaded from URDF via
        `moveit_commander`.
        This also initialises subscriber to joint states and publisher to joint trajectories.
        """
        self.robot_group_name = "panda_arm_hand"
        # Arm
        self.arm_group_name = "panda_arm"
        self.arm_joints = ["panda_joint1",
                           "panda_joint2",
                           "panda_joint3",
                           "panda_joint4",
                           "panda_joint5",
                           "panda_joint6",
                           "panda_joint7"]
        self.arm_links = ["panda_link0",
                          "panda_link1",
                          "panda_link2",
                          "panda_link3",
                          "panda_link4",
                          "panda_link5",
                          "panda_link6",
                          "panda_link7",
                          "panda_link8"]
        self.arm_base_link = self.arm_links[0]
        self.arm_end_effector = self.arm_links[-1]
        # Gripper
        self.gripper_group_name = "hand"
        self.gripper_joints = ["panda_finger_joint1",
                               "panda_finger_joint2"]
        self.gripper_links = ["panda_leftfinger",
                              "panda_rightfinger"]
        self.gripper_max_speed = 0.2

        # Publisher of trajectories
        self.joint_trajectory_pub = self.create_publisher(JointTrajectory,
                                                          "joint_trajectory", 1)

        self.use_separate_gripper_controller = separate_gripper_controller
        if separate_gripper_controller:
            self.gripper_trajectory_pub = self.create_publisher(JointTrajectory,
                                                                "gripper_trajectory", 1)

        # Subscriber of current joint states
        self.joint_state = JointState()
        self.joint_state_mutex = threading.Lock()
        self.joint_state_sub = self.create_subscription(JointState,
                                                        "joint_states",
                                                        self.joint_state_callback, 1)

        # Subscriber of joint trajectory progress
        self.joint_progress = 1.0
        self.joint_progress_cond = threading.Condition()
        self.joint_progress_sub = self.create_subscription(Float32,
                                                           "joint_trajectory_progress",
                                                           self.joint_progress_callback, 1)

    # gripper
    def init_gripper(self):
        """
        Initialise gripper interface.
        """

        # Gripper
        self.gripper_group_name = "hand"
        self.gripper_joints = ["panda_finger_joint1",
                               "panda_finger_joint2"]
        self.gripper_links = ["panda_leftfinger",
                              "panda_rightfinger"]
        self.gripper_max_speed = 0.2

        # Service client for IK
        self.plan_gripper_path_client = self.create_client(GetMotionPlan,
                                                           "plan_kinematic_path")
        while not self.plan_gripper_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Service [plan_kinematic_path] not currently available, waiting...")

        self.gripper_path_request = GetMotionPlan.Request()
        self.gripper_path_request.motion_plan_request.workspace_parameters.header.frame_id = \
            self.arm_base_link
        # self.gripper_path_request.motion_plan_request.workspace_parameters.header.stamp = \
        # "Set during request"
        self.gripper_path_request.motion_plan_request.workspace_parameters.min_corner.x = -0.855
        self.gripper_path_request.motion_plan_request.workspace_parameters.min_corner.y = -0.855
        self.gripper_path_request.motion_plan_request.workspace_parameters.min_corner.z = -0.36
        self.gripper_path_request.motion_plan_request.workspace_parameters.max_corner.x = 0.855
        self.gripper_path_request.motion_plan_request.workspace_parameters.max_corner.y = 0.855
        self.gripper_path_request.motion_plan_request.workspace_parameters.max_corner.z = 1.19
        # self.gripper_path_request.motion_plan_request.start_state = "Ignored"
        self.gripper_path_request.motion_plan_request.goal_constraints = \
            [Constraints()]
        (self.gripper_path_request.motion_plan_request.goal_constraints[0].joint_constraints
         .append(JointConstraint()))
        (self.gripper_path_request.motion_plan_request.goal_constraints[0].joint_constraints[0]
         .joint_name) = self.gripper_joints[0]
        # (self.gripper_path_request.motion_plan_request.goal_constraints[0].joint_constraints[0]
        # .position) = "Set during request"
        # (self.gripper_path_request.motion_plan_request.goal_constraints[0].joint_constraints[0]
        # .tolerance_above) = "Ignored"
        # (self.gripper_path_request.motion_plan_request.goal_constraints[0].joint_constraints[0]
        # .tolerance_below) = "Ignored"
        self.gripper_path_request.motion_plan_request.goal_constraints[
            0].joint_constraints[0].weight = 1.0
        # self.gripper_path_request.motion_plan_request.path_constraints = "Ignored"
        # self.gripper_path_request.motion_plan_request.trajectory_constraints = "Ignored"
        # self.gripper_path_request.motion_plan_request.reference_trajectories = "Ignored"
        # self.gripper_path_request.motion_plan_request.planner_id = "Ignored"
        self.gripper_path_request.motion_plan_request.group_name = self.gripper_group_name
        self.gripper_path_request.motion_plan_request.num_planning_attempts = 1
        self.gripper_path_request.motion_plan_request.allowed_planning_time = 0.1
        # self.gripper_path_request.motion_plan_request.max_velocity_scaling_factor = \
        # "Set during request"
        self.gripper_path_request.motion_plan_request.max_acceleration_scaling_factor = 0.0
        # self.gripper_path_request.motion_plan_request.cartesian_speed_end_effector_link = \
        # "Ignored"
        # self.gripper_path_request.motion_plan_request.max_cartesian_speed = "Ignored"

    def gripper_plan_path(self, width, speed) -> JointTrajectory:
        """
        Plan kinematic path for gripper.
        """
        self.gripper_path_request.motion_plan_request.max_velocity_scaling_factor = \
            float(speed/self.gripper_max_speed)
        (self.gripper_path_request.motion_plan_request.goal_constraints[0].
         joint_constraints[0].position) = float(width/2)

        self.plan_gripper_path_client.wait_for_service()
        response = self.plan_gripper_path_client.call(
            self.gripper_path_request)

        joint_trajectory = response.motion_plan_response.trajectory.joint_trajectory

        # Mirror motion on the second finger (might be more efficient than double planning)
        joint_trajectory.joint_names.append(self.gripper_joints[1])
        for i in range(len(joint_trajectory.points)):
            if joint_trajectory.points[i].positions:
                (joint_trajectory.points[i].positions
                 .append(joint_trajectory.points[i].positions[0]))
            if joint_trajectory.points[i].velocities:
                (joint_trajectory.points[i].velocities
                 .append(joint_trajectory.points[i].velocities[0]))
            if joint_trajectory.points[i].accelerations:
                (joint_trajectory.points[i].accelerations
                 .append(joint_trajectory.points[i].accelerations[0]))
            if joint_trajectory.points[i].effort:
                (joint_trajectory.points[i].effort
                 .append(joint_trajectory.points[i].effort[0]))

        return joint_trajectory

    def grasp(self, width=0.0, speed=0.2, force=20.0, force_start=0.75) -> bool:
        """
        Close gripper. Argument `force_start` (0.0,1.0] can be used to indicate point of the
        trajectory at which force will start being applied.
        """
        joint_trajectory = self.gripper_plan_path(width, speed)

        if not joint_trajectory.points:
            return False

        # Start slowly applying force in the last (1-force_start)% of the trajectory
        force_index_end = len(joint_trajectory.points)
        force_index_start = round(force_start*force_index_end)
        force_range = force_index_end - force_index_start
        for i in range(force_index_start, force_index_end):
            # Scale the applied force linearly with the index
            scaling_factor = ((i+1)-force_index_start) / force_range
            applied_force = scaling_factor * force
            # Closing direction is in negative directions
            joint_trajectory.points[i].effort = [-applied_force, -applied_force]

        return self.execute(joint_trajectory, is_gripper=True)

    def move(self, width=0.08, speed=0.2) -> bool:
        """
        Move fingers to the selected width.
        """
        joint_trajectory = self.gripper_plan_path(width, speed)

        if not joint_trajectory.points:
            return False

        # Make sure no force is applied anymore
        joint_trajectory.points[0].effort = [0.0, 0.0]

        return self.execute(joint_trajectory, is_gripper=True)



