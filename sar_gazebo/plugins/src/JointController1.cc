#include "JointController1.hh"
#include <iostream>
#include <ignition/plugin/Register.hh>

#include <ignition/msgs/actuators.pb.h>
#include <ignition/msgs/double.pb.h>

#include <string>
#include <vector>

#include <ignition/common/Profiler.hh>
#include <ignition/math/PID.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Actuators.hh"
#include "ignition/gazebo/components/JointForceCmd.hh"
#include "ignition/gazebo/components/JointVelocity.hh"
#include "ignition/gazebo/components/JointVelocityCmd.hh"
#include "ignition/gazebo/Model.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::JointControllerPrivate
{
  /// \brief Callback for velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnCmdVel(const msgs::Double &_msg);

  /// \brief Callback for actuator velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnActuatorVel(const msgs::Actuators &_msg);

  /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief Joint Entity
  public: std::vector<Entity> jointEntities;

  /// \brief Joint name
  public: std::vector<std::string> jointNames;

  /// \brief Commanded joint velocity
  public: double jointVelCmd{0.0};

  /// \brief Index of velocity actuator.
  public: int actuatorNumber = 0;

  /// \brief mutex to protect jointVelCmd
  public: std::mutex jointVelCmdMutex;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief True if using Actuator msg to control joint velocity.
  public: bool useActuatorMsg{false};

  /// \brief True if force commands are internally used to keep the target
  /// velocity.
  public: bool useForceCommands{false};

  /// \brief Velocity PID controller.
  public: math::PID velPid;
};

JointController::JointController()
  : dataPtr(std::make_unique<JointControllerPrivate>())
{
}

void JointController::Configure(const ignition::gazebo::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         ignition::gazebo::EntityComponentManager &_ecm,
                         ignition::gazebo::EventManager &_eventMgr)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "JointController plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }


  // Get params from SDF
  auto sdfElem = _sdf->FindElement("joint_name");
  while (sdfElem)
  {
    if (!sdfElem->Get<std::string>().empty())
    {
      this->dataPtr->jointNames.push_back(sdfElem->Get<std::string>());
    }
    else
    {
      ignerr << "<joint_name> provided but is empty." << std::endl;
    }
    sdfElem = sdfElem->GetNextElement("joint_name");
  }
  if (this->dataPtr->jointNames.empty())
  {
    ignerr << "Failed to get any <joint_name>." << std::endl;
    return;
  }


  if (_sdf->HasElement("initial_velocity"))
  {
    this->dataPtr->jointVelCmd = _sdf->Get<double>("initial_velocity");
    ignmsg << "Joint velocity initialized to ["
           << this->dataPtr->jointVelCmd << "]" << std::endl;
  }

  if (_sdf->HasElement("use_force_commands") &&
      _sdf->Get<bool>("use_force_commands"))
  {
    this->dataPtr->useForceCommands = true;

    // PID parameters
    double p         = _sdf->Get<double>("p_gain",     1.0).first;
    double i         = _sdf->Get<double>("i_gain",     0.0).first;
    double d         = _sdf->Get<double>("d_gain",     0.0).first;
    double iMax      = _sdf->Get<double>("i_max",      1.0).first;
    double iMin      = _sdf->Get<double>("i_min",     -1.0).first;
    double cmdMax    = _sdf->Get<double>("cmd_max",    1000.0).first;
    double cmdMin    = _sdf->Get<double>("cmd_min",   -1000.0).first;
    double cmdOffset = _sdf->Get<double>("cmd_offset", 0.0).first;

    this->dataPtr->velPid.Init(p, i, d, iMax, iMin, cmdMax, cmdMin, cmdOffset);

    igndbg << "[JointController] Force mode with parameters:" << std::endl;
    igndbg << "p_gain: ["     << p         << "]"             << std::endl;
    igndbg << "i_gain: ["     << i         << "]"             << std::endl;
    igndbg << "d_gain: ["     << d         << "]"             << std::endl;
    igndbg << "i_max: ["      << iMax      << "]"             << std::endl;
    igndbg << "i_min: ["      << iMin      << "]"             << std::endl;
    igndbg << "cmd_max: ["    << cmdMax    << "]"             << std::endl;
    igndbg << "cmd_min: ["    << cmdMin    << "]"             << std::endl;
    igndbg << "cmd_offset: [" << cmdOffset << "]"             << std::endl;
  }
  else
  {
    igndbg << "[JointController] Velocity mode" << std::endl;
  }

  if (_sdf->HasElement("use_actuator_msg") &&
    _sdf->Get<bool>("use_actuator_msg"))
  {
    if (_sdf->HasElement("actuator_number"))
    {
      this->dataPtr->actuatorNumber =
        _sdf->Get<int>("actuator_number");
      this->dataPtr->useActuatorMsg = true;
    }
    else
    {
      ignerr << "Please specify an actuator_number" <<
        "to use Actuator velocity message control." << std::endl;
    }
  }

  // Subscribe to commands
  std::string topic;
  if ((!_sdf->HasElement("sub_topic")) && (!_sdf->HasElement("topic"))
    && (!this->dataPtr->useActuatorMsg))
  {
    topic = transport::TopicUtils::AsValidTopic("/model/" +
        this->dataPtr->model.Name(_ecm) + "/joint/" +
        this->dataPtr->jointNames[0] + "/cmd_vel");
    if (topic.empty())
    {
      ignerr << "Failed to create topic for joint ["
            << this->dataPtr->jointNames[0]
            << "]" << std::endl;
      return;
    }
  }

  if ((!_sdf->HasElement("sub_topic")) && (!_sdf->HasElement("topic"))
    && (this->dataPtr->useActuatorMsg))
  {
    topic = transport::TopicUtils::AsValidTopic("/actuators");
    if (topic.empty())
    {
      ignerr << "Failed to create Actuator topic for joint ["
            << this->dataPtr->jointNames[0]
            << "]" << std::endl;
      return;
    }
  }


  if (_sdf->HasElement("sub_topic"))
  {
    topic = transport::TopicUtils::AsValidTopic("/model/" +
      this->dataPtr->model.Name(_ecm) + "/" +
        _sdf->Get<std::string>("sub_topic"));

    if (topic.empty())
    {
      ignerr << "Failed to create topic from sub_topic [/model/"
             << this->dataPtr->model.Name(_ecm) << "/"
             << _sdf->Get<std::string>("sub_topic")
             << "]" << " for joint [" << this->dataPtr->jointNames[0]
             << "]" << std::endl;
      return;
    }
  }
  if (_sdf->HasElement("topic"))
  {
    topic = transport::TopicUtils::AsValidTopic(
        _sdf->Get<std::string>("topic"));

    if (topic.empty())
    {
      ignerr << "Failed to create topic [" << _sdf->Get<std::string>("topic")
             << "]" << " for joint [" << this->dataPtr->jointNames[0]
             << "]" << std::endl;
      return;
    }
  }

  if (this->dataPtr->useActuatorMsg)
  {
    this->dataPtr->node.Subscribe(topic,
      &JointControllerPrivate::OnActuatorVel,
      this->dataPtr.get());

    ignmsg << "JointController subscribing to Actuator messages on [" << topic
         << "]" << std::endl;
  }
  else
  {
    this->dataPtr->node.Subscribe(topic,
      &JointControllerPrivate::OnCmdVel,
      this->dataPtr.get());

    ignmsg << "JointController subscribing to Double messages on [" << topic
         << "]" << std::endl;
  }
}

void JointController::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                         ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("JointController::PreUpdate");
  
  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }
  // If the joints haven't been identified yet, look for them
  if (this->dataPtr->jointEntities.empty())
  {
    bool warned{false};
    for (const std::string &name : this->dataPtr->jointNames)
    {
      Entity joint = this->dataPtr->model.JointByName(_ecm, name);
      if (joint != kNullEntity)
      {
        this->dataPtr->jointEntities.push_back(joint);
      }
      else if (!warned)
      {
        ignwarn << "Failed to find joint [" << name << "]" << std::endl;
        warned = true;
      }
    }
  }
  if (this->dataPtr->jointEntities.empty())
    return;

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  // Create joint velocity component if one doesn't exist
  auto jointVelComp = _ecm.Component<components::JointVelocity>(
      this->dataPtr->jointEntities[0]);
  if (!jointVelComp)
  {
    _ecm.CreateComponent(this->dataPtr->jointEntities[0],
        components::JointVelocity());
  }


  // We just created the joint velocity component, give one iteration for the
  // physics system to update its size
  if (jointVelComp == nullptr || jointVelComp->Data().empty())
    return;


  double targetVel;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->jointVelCmdMutex);
    targetVel = this->dataPtr->jointVelCmd;
  }
  
  double error = jointVelComp->Data().at(0) - targetVel;


  // Force mode.
  if ((this->dataPtr->useForceCommands) &&
      (!jointVelComp->Data().empty()))
  {
    for (Entity joint : this->dataPtr->jointEntities)
    {
      // Update force command.
      double force = this->dataPtr->velPid.Update(error, _info.dt);

      auto forceComp =
          _ecm.Component<components::JointForceCmd>(joint);
      if (forceComp == nullptr)
      {
        _ecm.CreateComponent(joint,
                            components::JointForceCmd({force}));
      }
      else
      {
        *forceComp = components::JointForceCmd({force});
      }
    }
  }
  // Velocity mode.
  else if (!this->dataPtr->useForceCommands)
  {
    // Update joint velocity
    for (Entity joint : this->dataPtr->jointEntities)
    {
      auto vel = _ecm.Component<components::JointVelocityCmd>(joint);

      if (vel == nullptr)
      {
        _ecm.CreateComponent(
            joint, components::JointVelocityCmd({targetVel}));
      }
      else
      {
        *vel = components::JointVelocityCmd({targetVel});
      }
    }
  }

}

//////////////////////////////////////////////////
void JointControllerPrivate::OnCmdVel(const msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(this->jointVelCmdMutex);
  this->jointVelCmd = _msg.data();
}

void JointControllerPrivate::OnActuatorVel(const msgs::Actuators &_msg)
{
  std::lock_guard<std::mutex> lock(this->jointVelCmdMutex);
  if (this->actuatorNumber > _msg.velocity_size() - 1)
  {
    ignerr << "You tried to access index " << this->actuatorNumber
      << " of the Actuator velocity array which is of size "
      << _msg.velocity_size() << std::endl;
    return;
  }

  this->jointVelCmd = static_cast<double>(_msg.velocity(this->actuatorNumber));
}



IGNITION_ADD_PLUGIN(JointController,
                    ignition::gazebo::System,
                    JointController::ISystemConfigure,
                    JointController::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(JointController, "ignition::gazebo::systems::JointController")