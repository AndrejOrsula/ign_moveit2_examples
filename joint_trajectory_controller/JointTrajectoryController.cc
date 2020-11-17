#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointType.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>

#include <ignition/math/PID.hh>

#include <ignition/transport/Node.hh>
#include <ignition/msgs/joint_trajectory.pb.h>

#include "JointTrajectoryController.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief A single 1-axis joint that is controlled by JointTrajectoryController plugin
class ActuatedJoint
{
public:
  /// \brief Default contructor
  ActuatedJoint() = default;

  /// \brief Constructor that is aware of SDF configuration
  /// \param[in] _entity Entity of the joint
  /// \param[in] _sdf SDF reference used to obtain configuration for this joint
  /// \param[in] _jointIndex Index of the joint, used to determine what SDF parameters belong to it
  ActuatedJoint(const Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                const size_t &_jointIndex);

  /// \brief Setup components required for control of this joint
  /// \param[in,out] _ecm Ignition Entity Component Manager
  /// \return True if setup was successful, False if any of the components could not be created
  bool SetupComponents(ignition::gazebo::EntityComponentManager &_ecm) const;

  /// \brief Set target of the joint that the controller will attemp to reach
  /// \param[in] _targetPoint Targets of all controlled joint
  /// \param[in] _jointIndex Index of the joint, used to determine what index of `_targetPoint` to use
  void SetTarget(const ignition::msgs::JointTrajectoryPoint &_targetPoint,
                 const size_t &_jointIndex);

  /// \brief Update command force that is applied on the joint
  /// \param[in,out] _ecm Ignition Entity Component Manager
  /// \param[in] _dt Time difference to update for
  void Update(ignition::gazebo::EntityComponentManager &_ecm,
              const std::chrono::steady_clock::duration &_dt);

  /// \brief Reset the position and velocity PID error on the joint
  void ResetPIDs();

  /// \brief Entity of the joint
  Entity entity;

  /// \brief Target state that the joint controller should reach
  struct TargetState
  {
    /// \brief Target position of the joint
    double position;
    /// \brief Target position of the joint
    double velocity;
    /// \brief Target acceleration of the joint
    /// \attention Acceleration control is NOT implemented at the moment
    double acceleration;
    /// \brief Target force or torque of the joint
    double effort;
  } target;

  /// \brief Initial position of the joint
  double initialPosition;

  /// \brief List of PID controllers used for the control of this actuated joint
  struct PIDs
  {
    /// \brief Position PID controller
    ignition::math::PID position;
    /// \brief Velocity PID controller
    ignition::math::PID velocity;
  } pids;
};

/// \brief Information about trajectory that is followed by JointTrajectoryController plugin
class Trajectory
{
public:
  /// \brief Update index of trajectory points, such that it directs to a point that needs to be currently followed
  /// \param[in] _simTime Current simulation time
  /// \return True if index of the trajectory point was updated, False otherwise
  bool UpdateCurrentPoint(const std::chrono::steady_clock::duration &_simTime);

  /// \brief Determine if the trajectory goal was reached
  /// \return True if trajectory goal was reached, False otherwise
  bool IsGoalReached() const;

  /// \brief Compute completeness of the current trajectory
  /// \return Fraction of the completed points in range [0.0, 1.0]
  double ComputeCompleteness() const;

  /// \brief Reset trajectory internals, i.e. clean list of joint names, points and reset index of the current point
  void Reset();

  /// \brief Status of the trajectory
  enum TrajectoryStatus
  {
    /// \brief Trajectory is new and needs to be configure on the next update loop
    New,
    /// \brief Trajectory is currently being followed
    Active,
    /// \brief Trajectory goal is reached
    Reached,
  } progress;

  /// \brief Start time of trajectory
  std::chrono::steady_clock::duration startTime;

  /// \brief Index of the current trajectory point
  unsigned int pointIndex;

  /// \brief Ordered joints that need to be actuated to follow the current trajectory
  std::vector<std::string> jointNames;

  /// \brief Trajectory defined in terms of temporal points, whose members are ordered according to `jointNames`
  std::vector<ignition::msgs::JointTrajectoryPoint> points;
};

/// \brief Private data of the JointTrajectoryController plugin
class ignition::gazebo::systems::JointTrajectoryControllerPrivate
{
public:
  /// \brief Callback for joint trajectory subscription
  /// \param[in] _msg A new message describing a joint trajectory that needs to be followed
  void JointTrajectoryCallback(const ignition::msgs::JointTrajectory &_msg);

  /// \brief Configure a single joint so that it can be actuated to follow a trajectory
  /// \param[in] _entity Entity of the joint
  /// \param[in] _sdf SDF reference used to obtain configuration for this joint
  /// \param[in] _ecm Ignition Entity Component Manager
  /// \param[in] _enabledJoints List of all joints that should be enabled based on SDF configuration
  void ConfigureJoint(const Entity &_entity,
                      const std::shared_ptr<const sdf::Element> &_sdf,
                      const ignition::gazebo::EntityComponentManager &_ecm,
                      const std::vector<std::string> &_enabledJoints);

  /// \brief Reset internals of the plugin, without affecting already created components
  void Reset();

  /// \brief Ignition communication node
  transport::Node node;

  /// \brief Map of actuated joints, where first is the name of the joint
  std::map<std::string, ActuatedJoint> actuatedJoints;

  /// \brief Mutex projecting trajectory
  std::mutex trajectoryMutex;

  /// \brief Information about trajectory that should be followed
  Trajectory trajectory;

  /// \brief Flag that determines whether to use message header timestamp as the trajectory start,
  /// where simulation time at the beginning of execution is used otherwise
  bool useHeaderStartTime;

  /// \brief Flag that determines if all compenents required for control are already setup
  bool componentSetupFinished;
};

////////////////////////
/// Helper Functions ///
////////////////////////

/// \brief Parse separated parameters from a string
/// \param[in] _input A string that has multiple separated parameters, e.g. with whitespace
/// \return Vector of all T-typed parameters contained in the input
template <typename T>
std::vector<T> ParseVectorParam(const std::string &_input)
{
  std::istringstream iss(_input);
  std::vector<T> output;
  T value;

  while (iss >> value)
  {
    output.push_back(value);
  }
  return output;
}

/// \brief Return value from `_vec` at `_index`, or `_default` if `_vec` is not large enough
/// \param[in] _vec Vector that contains desired value of type T
/// \param[in] _index Index at which the value is stored
/// \param[in] _default Default value of type T
/// \return Value from `_vec` at `_index`, or `_default` if `_vec` is not large enough
template <typename T>
T GetNthOrDefault(const std::vector<T> &_vec,
                  const size_t &_index,
                  const T &_default)
{
  if (_index < _vec.size())
  {
    return _vec[_index];
  }
  else
  {
    return _default;
  }
}

/////////////////////////////////
/// JointTrajectoryController ///
/////////////////////////////////

JointTrajectoryController::JointTrajectoryController() : dataPtr(std::make_unique<JointTrajectoryControllerPrivate>())
{
}

void JointTrajectoryController::Configure(const Entity &_entity,
                                          const std::shared_ptr<const sdf::Element> &_sdf,
                                          EntityComponentManager &_ecm,
                                          EventManager & /*_eventManager*/)
{
  // Make sure the controller is attached to a valid plugin
  const auto model = Model(_entity);
  if (!model.Valid(_ecm))
  {
    ignerr << "[JointTrajectoryController] Failed to initialize because [" << model.Name(_ecm) << "(Entity="
           << _entity << ")] is not a model. Please make sure that JointTrajectoryController is attached to a valid model.\n";
    return;
  }
  ignmsg << "[JointTrajectoryController] Setting up controller for [" << model.Name(_ecm) << "(Entity=" << _entity << ")].\n";

  // Get list of enabled joints
  // If empty, enable all joints
  const auto enabledJoints = ParseVectorParam<std::string>(_sdf->Get<std::string>("joint_names"));

  // Iterate over all joints under the model entity and configure them
  for (const auto &jointEntity : _ecm.ChildrenByComponents(_entity, components::Joint()))
  {
    this->dataPtr->ConfigureJoint(jointEntity, _sdf, _ecm, enabledJoints);
  }

  // Make sure at least one joint is configured
  if (this->dataPtr->actuatedJoints.empty())
  {
    ignerr << "[JointTrajectoryController] Failed to initialize because [" << model.Name(_ecm) << "(Entity=" << _entity
           << ")] has no supported joints.\n";
    return;
  }

  // Get additional parameters from SDF
  if (_sdf->HasAttribute("use_header_start_time"))
  {
    this->dataPtr->useHeaderStartTime = _sdf->Get<bool>("use_header_start_time");
  }
  else
  {
    this->dataPtr->useHeaderStartTime = false;
  }

  // Subscribe to joint trajectory commands
  auto topic = _sdf->Get<std::string>("topic");
  if (topic.empty())
  {
    topic = "/model/" + model.Name(_ecm) + "/joint_trajectory";
    ignmsg << "[JointTrajectoryController] No topic specified, defaulting to [" << topic << "].\n";
  }
  else
  {
    if (topic[0] != '/') {
      topic.insert(0, "/");
    }
    ignmsg << "[JointTrajectoryController] Topic set to [" << topic << "].\n";
  }
  this->dataPtr->node.Subscribe(topic, &JointTrajectoryControllerPrivate::JointTrajectoryCallback, this->dataPtr.get());
}

void JointTrajectoryController::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                                          ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("JointTrajectoryController::PreUpdate");

  // Create required components for each joint (only once)
  if (!this->dataPtr->componentSetupFinished)
  {
    for (auto &actuatedJoint : this->dataPtr->actuatedJoints)
    {
      ActuatedJoint *joint = &actuatedJoint.second;
      if (!joint->SetupComponents(_ecm))
      {
        return;
      }
    }
    this->dataPtr->componentSetupFinished = true;
  }

  // Reset plugin if jump back in time is detected
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignmsg << "[JointTrajectoryController] Resetting plugin because jump back in time ["
           << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
           << " s] was detected.\n";
    this->dataPtr->Reset();
  }

  // Nothing else to do if paused
  if (_info.paused)
  {
    return;
  }

  // Update joint targets based on the current trajectory
  {
    auto isTargetUpdateRequired = false;

    // Lock mutex before accessing trajectory
    std::lock_guard<std::mutex> lock(this->dataPtr->trajectoryMutex);

    if (this->dataPtr->trajectory.progress == Trajectory::New)
    {
      // Set trajectory start time if not set before
      if (this->dataPtr->trajectory.startTime.count() == 0)
      {
        this->dataPtr->trajectory.startTime = _info.simTime;
        this->dataPtr->trajectory.progress = Trajectory::Active;
      }

      // If the new trajectory has no points, consider it reached
      if (this->dataPtr->trajectory.points.empty())
      {
        this->dataPtr->trajectory.progress = Trajectory::Reached;
      }

      // Reset PID errors of the joints affected by the new trajectory
      for (const auto &jointName : this->dataPtr->trajectory.jointNames)
      {
        if (this->dataPtr->actuatedJoints.count(jointName) == 0)
        {
          ignwarn << "[JointTrajectoryController] Trajectory contains unconfigured joint ["
                  << jointName << "].\n";
          continue;
        }
        auto *joint = &this->dataPtr->actuatedJoints[jointName];
        joint->ResetPIDs();
      }

      // Update is always needed for a new trajectory
      isTargetUpdateRequired = true;
    }

    if (this->dataPtr->trajectory.progress == Trajectory::Active)
    {
      // Determine what point needs to be reached at the current time
      if (this->dataPtr->trajectory.UpdateCurrentPoint(_info.simTime))
      {
        // Update is needed if point was updated
        isTargetUpdateRequired = true;
      }
    }

    // Update the target for each joint that is defined in the trajectory, if needed
    if (isTargetUpdateRequired && this->dataPtr->trajectory.progress != Trajectory::Reached)
    {
      const auto targetPoint = this->dataPtr->trajectory.points[this->dataPtr->trajectory.pointIndex];
      for (auto jointIndex = 0u; jointIndex < this->dataPtr->trajectory.jointNames.size(); ++jointIndex)
      {
        const auto jointName = this->dataPtr->trajectory.jointNames[jointIndex];
        if (this->dataPtr->actuatedJoints.count(jointName) == 0)
        {
          // Unconfigured joint is already logged
          continue;
        }
        auto *joint = &this->dataPtr->actuatedJoints[jointName];
        joint->SetTarget(targetPoint, jointIndex);
      }

      // If there are no more points after the current one, set the trajectory to Reached
      if (this->dataPtr->trajectory.IsGoalReached())
      {
        this->dataPtr->trajectory.progress = Trajectory::Reached;
      }
    }
  }

  // Control loop
  for (auto &actuatedJoint : this->dataPtr->actuatedJoints)
  {
    auto *joint = &actuatedJoint.second;
    joint->Update(_ecm, _info.dt);
  }
}

////////////////////////////////////////
/// JointTrajectoryControllerPrivate ///
////////////////////////////////////////

void JointTrajectoryControllerPrivate::ConfigureJoint(const Entity &_entity,
                                                      const std::shared_ptr<const sdf::Element> &_sdf,
                                                      const ignition::gazebo::EntityComponentManager &_ecm,
                                                      const std::vector<std::string> &_enabledJoints)
{
  const auto jointName = _ecm.Component<components::Name>(_entity)->Data();
  auto jointIndex = this->actuatedJoints.size();

  // Ignore duplicate joints
  for (const auto &actuatedJoint : this->actuatedJoints)
  {
    if (actuatedJoint.second.entity == _entity)
    {
      ignwarn << "[JointTrajectoryController] Ignoring duplicate joint [" << jointName << "(Entity=" << _entity << ")]\".\n";
      continue;
    }
  }

  // Make sure the joint type is supported, i.e. it has a single actuated axis
  const auto *jointType = _ecm.Component<components::JointType>(_entity);
  switch (jointType->Data())
  {
  case sdf::JointType::PRISMATIC:
  case sdf::JointType::REVOLUTE:
  case sdf::JointType::CONTINUOUS:
  case sdf::JointType::GEARBOX:
  {
    // Supported joint type
    break;
  }
  case sdf::JointType::FIXED:
  {
    igndbg << "[JointTrajectoryController] Fixed joint [" << jointName << "(Entity=" << _entity << ")] is skipped.\n";
    return;
  }
  case sdf::JointType::REVOLUTE2:
  case sdf::JointType::SCREW:
  case sdf::JointType::BALL:
  case sdf::JointType::UNIVERSAL:
  {
    ignwarn << "[JointTrajectoryController] Joint [" << jointName << "(Entity=" << _entity << ")] is of unsupported type. "
            << "Only joints with a single axis are supported.\n";
    return;
  }
  default:
  {
    ignwarn << "[JointTrajectoryController] Joint [" << jointName << "(Entity=" << _entity << ")] is of unknown type.\n";
    return;
  }
  }

  // Skip if joint is not enabled
  if (!_enabledJoints.empty())
  {
    const auto it = std::find(_enabledJoints.begin(), _enabledJoints.end(), jointName);
    if (it == _enabledJoints.end())
    {
      ignmsg << "[JointTrajectoryController] Ignoring disabled joint [" << jointName << "(Entity=" << _entity << ")]\".\n";
      return;
    }
    // If enabled, update joint index to match the order specified in SDF
    jointIndex = std::distance(_enabledJoints.begin(), it);
  }

  // Create a new actuated joint
  this->actuatedJoints[jointName] = ActuatedJoint(_entity, _sdf, jointIndex);
  ignmsg << "[JointTrajectoryController] Configured joint [" << jointName << "(Entity=" << _entity << ")].\n";
}

void JointTrajectoryControllerPrivate::JointTrajectoryCallback(const ignition::msgs::JointTrajectory &_msg)
{
  // Make sure the message is valid
  if (_msg.joint_names_size() == 0)
  {
    ignwarn << "[JointTrajectoryController] JointTrajectory message does not contain any joint names.\n";
    return;
  }

  // Lock mutex guarding the trajectory
  std::lock_guard<std::mutex> lock(this->trajectoryMutex);

  if (this->trajectory.progress != Trajectory::Reached)
  {
    ignwarn << "[JointTrajectoryController] A new JointTrajectory message was received while executing a previous trajectory.\n";
    // TODO: Handle this behaviour properly
  }

  // Get start time of the trajectory from message header if desired
  // If not enabled or there is no header, set start time to 0 and determine it later based on simTime
  if (this->useHeaderStartTime && _msg.has_header())
  {
    if (_msg.header().has_stamp())
    {
      this->trajectory.startTime = std::chrono::seconds(_msg.header().stamp().sec()) +
                                   std::chrono::nanoseconds(_msg.header().stamp().nsec());
    }
  }
  else
  {
    this->trajectory.startTime = std::chrono::nanoseconds(0);
  }

  // Reset for a new trajectory
  this->trajectory.Reset();

  // Extract joint names and points
  for (const auto &joint_name : _msg.joint_names())
  {
    this->trajectory.jointNames.push_back(joint_name);
  }
  for (const auto &point : _msg.points())
  {
    this->trajectory.points.push_back(point);
  }
}

void JointTrajectoryControllerPrivate::Reset()
{
  for (auto &actuatedJoint : this->actuatedJoints)
  {
    auto *joint = &actuatedJoint.second;
    // Reset joint targets
    joint->target.position = joint->initialPosition;
    joint->target.velocity = 0.0;
    joint->target.acceleration = 0.0;
    joint->target.effort = 0.0;

    // Reset PIDs
    joint->ResetPIDs();
  }

  // Reset trajectory
  this->trajectory.Reset();
}

/////////////////////
/// ActuatedJoint ///
/////////////////////

ActuatedJoint::ActuatedJoint(const Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             const size_t &_jointIndex)
{
  this->entity = _entity;

  this->initialPosition = GetNthOrDefault(ParseVectorParam<double>(_sdf->Get<std::string>("initial_joint_positions")), _jointIndex, 0.0);
  this->target.position = this->initialPosition;
  this->target.velocity = 0.0;
  this->target.acceleration = 0.0;
  this->target.effort = 0.0;

  this->pids.position = ignition::math::PID(
      GetNthOrDefault(ParseVectorParam<double>(_sdf->Get<std::string>("position_p_gain")), _jointIndex, 0.0),
      GetNthOrDefault(ParseVectorParam<double>(_sdf->Get<std::string>("position_i_gain")), _jointIndex, 0.0),
      GetNthOrDefault(ParseVectorParam<double>(_sdf->Get<std::string>("position_d_gain")), _jointIndex, 0.0),
      GetNthOrDefault(ParseVectorParam<double>(_sdf->Get<std::string>("position_i_max")), _jointIndex, -1.0),
      GetNthOrDefault(ParseVectorParam<double>(_sdf->Get<std::string>("position_i_min")), _jointIndex, 0.0),
      GetNthOrDefault(ParseVectorParam<double>(_sdf->Get<std::string>("position_cmd_min")), _jointIndex, -1.0),
      GetNthOrDefault(ParseVectorParam<double>(_sdf->Get<std::string>("position_cmd_max")), _jointIndex, 0.0),
      GetNthOrDefault(ParseVectorParam<double>(_sdf->Get<std::string>("position_cmd_offset")), _jointIndex, 0.0));

  this->pids.velocity = ignition::math::PID(
      GetNthOrDefault(ParseVectorParam<double>(_sdf->Get<std::string>("velocity_p_gain")), _jointIndex, 0.0),
      GetNthOrDefault(ParseVectorParam<double>(_sdf->Get<std::string>("velocity_i_gain")), _jointIndex, 0.0),
      GetNthOrDefault(ParseVectorParam<double>(_sdf->Get<std::string>("velocity_d_gain")), _jointIndex, 0.0),
      GetNthOrDefault(ParseVectorParam<double>(_sdf->Get<std::string>("velocity_i_max")), _jointIndex, -1.0),
      GetNthOrDefault(ParseVectorParam<double>(_sdf->Get<std::string>("velocity_i_min")), _jointIndex, 0.0),
      GetNthOrDefault(ParseVectorParam<double>(_sdf->Get<std::string>("velocity_cmd_min")), _jointIndex, -1.0),
      GetNthOrDefault(ParseVectorParam<double>(_sdf->Get<std::string>("velocity_cmd_max")), _jointIndex, 0.0),
      GetNthOrDefault(ParseVectorParam<double>(_sdf->Get<std::string>("velocity_cmd_offset")), _jointIndex, 0.0));
}

bool ActuatedJoint::SetupComponents(ignition::gazebo::EntityComponentManager &_ecm) const
{
  const auto jointName = _ecm.Component<components::Name>(this->entity)->Data();

  // Create JointPosition component if one does not exist
  auto jointPositionComponent = _ecm.Component<components::JointPosition>(this->entity);
  if (jointPositionComponent == nullptr)
  {
    _ecm.CreateComponent(this->entity, components::JointPosition());
  }
  if (jointPositionComponent == nullptr)
  {
    ignwarn << "[JointTrajectoryController] Cannot create JointPosition component for a joint ["
            << jointName << "(Entity=" << this->entity << ")].\n";
    return false;
  }

  // Create JointVelocity component if one does not exist
  auto jointVelocityComponent = _ecm.Component<components::JointVelocity>(this->entity);
  if (jointVelocityComponent == nullptr)
  {
    _ecm.CreateComponent(this->entity, components::JointVelocity());
  }
  if (jointVelocityComponent == nullptr)
  {
    ignwarn << "[JointTrajectoryController] Cannot create JointVelocity component for a joint ["
            << jointName << "(Entity=" << this->entity << ")].\n";
    return false;
  }

  // Create JointForceCmd component if one does not exist
  auto jointForceCmdComponent = _ecm.Component<components::JointForceCmd>(this->entity);
  if (jointForceCmdComponent == nullptr)
  {
    _ecm.CreateComponent(this->entity, components::JointForceCmd({0.0}));
  }
  if (jointForceCmdComponent == nullptr)
  {
    ignwarn << "[JointTrajectoryController] Cannot create JointForceCmd component for a joint ["
            << jointName << "(Entity=" << this->entity << ")].\n";
    return false;
  }

  return true;
}

void ActuatedJoint::SetTarget(const ignition::msgs::JointTrajectoryPoint &_targetPoint,
                              const size_t &_jointIndex)
{
  if ((signed)_jointIndex < _targetPoint.positions_size())
  {
    this->target.position = _targetPoint.positions(_jointIndex);
  }
  if ((signed)_jointIndex < _targetPoint.velocities_size())
  {
    this->target.velocity = _targetPoint.velocities(_jointIndex);
  }
  if ((signed)_jointIndex < _targetPoint.accelerations_size())
  {
    this->target.acceleration = _targetPoint.accelerations(_jointIndex);
  }
  if ((signed)_jointIndex < _targetPoint.effort_size())
  {
    this->target.effort = _targetPoint.effort(_jointIndex);
  }
}

void ActuatedJoint::Update(ignition::gazebo::EntityComponentManager &_ecm,
                           const std::chrono::steady_clock::duration &_dt)
{
  // Get JointPosition and JointVelocity components
  const auto jointPositionComponent = _ecm.Component<components::JointPosition>(this->entity);
  const auto jointVelocityComponent = _ecm.Component<components::JointVelocity>(this->entity);

  // Compute control errors and force for each PID controller
  double forcePosition = 0.0, forceVelocity = 0.0;
  if (!jointPositionComponent->Data().empty())
  {
    double errorPosition = jointPositionComponent->Data()[0] - this->target.position;
    forcePosition = this->pids.position.Update(errorPosition, _dt);
  }
  if (!jointVelocityComponent->Data().empty())
  {
    double errorVelocity = jointVelocityComponent->Data()[0] - this->target.velocity;
    forceVelocity = this->pids.velocity.Update(errorVelocity, _dt);
  }

  // Sum all forces
  const double force = forcePosition + forceVelocity + this->target.effort;

  // Get JointForceCmd component and apply command force
  auto jointForceCmdComponent = _ecm.Component<components::JointForceCmd>(this->entity);
  jointForceCmdComponent->Data()[0] = force;
}

void ActuatedJoint::ResetPIDs()
{
  this->pids.position.Reset();
  this->pids.velocity.Reset();
}

//////////////////
/// Trajectory ///
//////////////////

bool Trajectory::UpdateCurrentPoint(const std::chrono::steady_clock::duration &_simTime)
{
  bool isUpdated = false;

  const auto trajectoryTime = _simTime - this->startTime;
  while (true)
  {
    // Break if end of trajectory is reached (there are no more points after the current one)
    if (this->IsGoalReached())
    {
      break;
    }

    // Break if point needs to be followed
    const auto pointTSF = this->points[this->pointIndex].time_from_start();
    const auto pointTime = std::chrono::seconds(pointTSF.sec()) + std::chrono::nanoseconds(pointTSF.nsec());
    if (pointTime >= trajectoryTime)
    {
      break;
    }

    // Otherwise increment and try again (joint targets need to be updated)
    ++this->pointIndex;
    isUpdated = true;
  };

  // Return true if a point index was updated
  return isUpdated;
}

bool Trajectory::IsGoalReached() const
{
  if (this->pointIndex + 1 >= this->points.size())
  {
    return true;
  }
  else
  {
    return false;
  }
}

double Trajectory::ComputeCompleteness() const
{
  if (this->points.size() == 0)
  {
    return 1.0;
  }
  else
  {
    return ((double)this->pointIndex + 1) / (double)this->points.size();
  }
}

void Trajectory::Reset()
{
  this->progress = Trajectory::New;
  this->pointIndex = 0;
  this->jointNames.clear();
  this->points.clear();
}

// Register plugin
IGNITION_ADD_PLUGIN(JointTrajectoryController,
                    ignition::gazebo::System,
                    JointTrajectoryController::ISystemConfigure,
                    JointTrajectoryController::ISystemPreUpdate)
IGNITION_ADD_PLUGIN_ALIAS(JointTrajectoryController,
                          "ignition::gazebo::systems::JointTrajectoryController")
