/// C++ MoveIt2 interface for Ignition Gazebo that utilises move_group API to generate JointTrajectory, which is then
/// subsequently published in order to be executed by JointTrajectoryController Ignition plugin.
/// This set of node currently serves as an example and is configured for Franka Emika Panda robot.

////////////////////
/// DEPENDENCIES ///
////////////////////

// ROS 2
#include <rclcpp/rclcpp.hpp>

// ROS 2 Interface
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// MoveIt2
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

//////////////////
/// NAMESPACES ///
//////////////////

using namespace std::chrono_literals;

/////////////////
/// CONSTANTS ///
/////////////////

/// The name of the primary node
const std::string NODE_NAME = "ign_moveit2";
/// The name of node responsible for MoveIt2, separated to keep at individual thread
const std::string NODE_NAME_MOVEIT2_HANDLER = "ign_moveit2_handler";
/// Identifier of the planning group
const std::string PLANNING_GROUP = "panda_arm";
/// Topic that planned trajectory will be published to
const std::string JOINT_TRAJECTORY_TOPIC = "joint_trajectory";

/////////////////////////////
/// Node - MoveIt2Handler ///
/////////////////////////////

class MoveIt2Handler : public rclcpp::Node
{
public:
  /// Constructor
  MoveIt2Handler();

  /// Publisher of the trajectories
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
  /// Planning scene interface
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  /// Move group interface for the robot
  moveit::planning_interface::MoveGroupInterface move_group_;

  /// Set goal to target joint values
  bool set_joint_goal(const std::vector<double, std::allocator<double>> &_target_joint_state);
  /// Set goal to target pose
  bool set_pose_goal(const geometry_msgs::msg::PoseStamped &_target_pose);
  /// Set goal to named target
  bool set_named_target_goal(const std::string &_named_target);
  /// Plan trajectory with previously set target
  bool plan_trajectory();
};

MoveIt2Handler::MoveIt2Handler() : Node(NODE_NAME_MOVEIT2_HANDLER, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
                                   trajectory_publisher_(this->create_publisher<trajectory_msgs::msg::JointTrajectory>(JOINT_TRAJECTORY_TOPIC, 1)),
                                   planning_scene_interface_(this->get_name()),
                                   move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), PLANNING_GROUP)
{
  // Various trajectory parameters can be set here
  this->move_group_.setMaxAccelerationScalingFactor(0.5);
  this->move_group_.setMaxVelocityScalingFactor(0.5);
  RCLCPP_INFO(this->get_logger(), "Node initialised successfuly");
}

bool MoveIt2Handler::set_joint_goal(const std::vector<double, std::allocator<double>> &_target_joint_state)
{
  RCLCPP_INFO(this->get_logger(), "Setting goal to a custom joint values");
  return this->move_group_.setJointValueTarget(_target_joint_state);
}

bool MoveIt2Handler::set_pose_goal(const geometry_msgs::msg::PoseStamped &_target_pose)
{
  RCLCPP_INFO(this->get_logger(), "Setting goal to a custom pose");
  return this->move_group_.setPoseTarget(_target_pose);
}

bool MoveIt2Handler::set_named_target_goal(const std::string &_named_target)
{
  RCLCPP_INFO(this->get_logger(), "Setting goal to named target \"" + _named_target + "\"");
  return this->move_group_.setNamedTarget(_named_target);
}

bool MoveIt2Handler::plan_trajectory()
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  if (this->move_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(this->get_logger(), "Planning successful");
    trajectory_publisher_->publish(plan.trajectory_.joint_trajectory);
    return true;
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Planning failed");
    return false;
  }
}

//////////////////////////////
/// Node - IgnitionMoveIt2 ///
//////////////////////////////

class IgnitionMoveIt2 : public rclcpp::Node
{
public:
  /// Constructor
  IgnitionMoveIt2(std::shared_ptr<MoveIt2Handler> &moveit2_handler_);

private:
  /// Pointer to a node handling the interfacing with MoveIt2
  std::shared_ptr<MoveIt2Handler> moveit2_handler_;

  void run_example();
};

IgnitionMoveIt2::IgnitionMoveIt2(std::shared_ptr<MoveIt2Handler> &_moveit2_handler) : Node(NODE_NAME),
                                                                                      moveit2_handler_(_moveit2_handler)
{
  RCLCPP_INFO(this->get_logger(), "Node initialised successfuly");

  // Example
  this->run_example();
}

void IgnitionMoveIt2::run_example()
{
  auto target_joints = this->moveit2_handler_->move_group_.getCurrentJointValues();
  target_joints[0] = 1.5707963;
  target_joints[1] = -0.78539816;
  target_joints[2] = 1.5707963;
  target_joints[3] = 0.78539816;
  target_joints[4] = -1.5707963;
  target_joints[5] = 1.5707963;
  target_joints[6] = 0.78539816;
  this->moveit2_handler_->set_joint_goal(target_joints);
  RCLCPP_INFO(this->get_logger(), "Moving to joint goal");
  this->moveit2_handler_->plan_trajectory();

  // Wait until finished
  rclcpp::sleep_for(5s);

  auto target_pose = this->moveit2_handler_->move_group_.getCurrentPose();
  target_pose.pose.position.x = 0.5;
  target_pose.pose.position.y = 0.25;
  target_pose.pose.position.z = 0.75;
  target_pose.pose.orientation.x = 0.0;
  target_pose.pose.orientation.y = 0.0;
  target_pose.pose.orientation.z = 0.0;
  target_pose.pose.orientation.w = 1.0;
  this->moveit2_handler_->set_pose_goal(target_pose);
  RCLCPP_INFO(this->get_logger(), "Moving to pose goal");
  this->moveit2_handler_->plan_trajectory();
}

////////////
/// MAIN ///
////////////

/// Main function that initiates nodes of this process
/// Multi-threaded executor is utilised such that MoveIt2 thread is separated
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto moveit2_handler = std::make_shared<MoveIt2Handler>();
  auto ign_moveit2 = std::make_shared<IgnitionMoveIt2>(moveit2_handler);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(moveit2_handler);
  executor.add_node(ign_moveit2);
  executor.spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
