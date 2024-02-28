#include "Motor_Plugin.h"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::Motor_PluginPrivate
{
  /*// CONFIG PARAMS
  std::string SAR_Type;
  std::string SAR_Config;
  std::string Cam_Config;

  // GAZEBO POINTERS
  physics::WorldPtr World_Ptr;
  physics::ModelPtr Model_Ptr;
  physics::JointPtr Joint_Ptr;
  physics::LinkPtr Link_Ptr;*/


  // MOTOR PARAMETERS
  public: std::string Motor_Joint_Name;
  public: std::string Prop_Link_Name;

  // MOTOR PARAMETERS
  public: int Motor_Number;
  //public: int Turn_Direction;
  public: std::string turningDirection;
  
  //public: double Thrust_Coeff = 2.2e-8; // Thrust Coeff [N/rad/s]
  public: double Torque_Coeff = 0.00618; // Torque Coeff [N*m/rad/s]
  public: double C_tf = 0.00618; // Torque-Thrust Coeff [N*m/N]

  // FIRST ORDER FILTER BEHAVIOR
  public: float Thrust_input = 0.0f;  // Desired Thrust [N]
  public: double Tau_up = 0.05;              // Motor Time Constant (Up) [s]
  public: double Tau_down = 0.15;            // Motor Time Constant (Down) [s]
  public: double Sampling_time;
  public: double Prev_Sim_time = 0.0;
  public: double Prev_Thrust = 0.0;  

  // CACULATED VALUES
  public: double Thrust=0.1;              // Calculated Thrust [N]
  public: double Torque;              // Calculated Torque [N*m]
  //public: double Rot_Vel = 0.0f;      // Rotational Velocity [rad/s]
  //public: double Rot_Vel_Slowdown;    // Slowed-down Rotational Velocity [rad/s]

  /*// GAZEBO CONNECTIONS
  event::ConnectionPtr updateConnection;

  // ROS CONNECTIONS
  ros::NodeHandle nh;
  ros::Subscriber CTRL_Data_Sub = nh.subscribe<sar_msgs::CTRL_Data>("/CTRL/data", 1, &Motor_Plugin::CtrlData_Callback, this, ros::TransportHints().tcpNoDelay());
  ros::Publisher MS_Data_Pub = nh.advertise<sar_msgs::MS>("/SAR_Internal/MS",1);
  sar_msgs::MS Thrust_msg;*/

  public: std::vector<std::string> jointNames;
  public: std::vector<std::string> linkNames;
  public: Model model{kNullEntity};

  public: void UpdateForcesAndMoments();

  /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief Joint Entity
  public: std::vector<Entity> jointEntities;

  /// \brief Commanded joint velocity
  public: double jointVelCmd{0.0};
  public: double JointForceCmd{0.0};



  /// \brief mutex to protect jointVelCmd
  public: std::mutex jointVelCmdMutex;









};

Motor_Plugin::Motor_Plugin()
  : dataPtr(std::make_unique<Motor_PluginPrivate>())
{
}

void Motor_Plugin::Configure(const ignition::gazebo::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         ignition::gazebo::EntityComponentManager &_ecm,
                         ignition::gazebo::EventManager &_eventMgr)
{
  ignmsg << "Loading Motor_Plugin\n";
  
  //GRAB MODEL
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "Motor_Plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // GRAB MOTOR JOINT FROM SDF
  auto Motor_Joint_Name = _sdf->FindElement("Joint_Name");
  while (Motor_Joint_Name)
  {
    if (!Motor_Joint_Name->Get<std::string>().empty())
    {
      this->dataPtr->jointNames.push_back(Motor_Joint_Name->Get<std::string>());
    }
    else
    {
      ignerr << "<Joint_Name> provided but is empty." << std::endl;
    }
    Motor_Joint_Name = Motor_Joint_Name->GetNextElement("Joint_Name");
  }

  if (this->dataPtr->jointNames.empty())
  {
    ignerr << "Failed to get any <Joint_Name>." << std::endl;
    return;
  }


  // GRAB ROTOR LINK FROM SDF
  auto Prop_Link_Name = _sdf->FindElement("Link_Name");
  while (Prop_Link_Name)
  {
    if (!Prop_Link_Name->Get<std::string>().empty())
    {
      this->dataPtr->linkNames.push_back(Prop_Link_Name->Get<std::string>());
    }
    else
    {
      ignerr << "<Link_Name> provided but is empty." << std::endl;
    }
    Prop_Link_Name = Prop_Link_Name->GetNextElement("Link_Name");
  }

  if (this->dataPtr->linkNames.empty())
  {
    ignerr << "Failed to get any <Link_Name>!!!!!." << std::endl;
    return;
  }

  // COLLECT OTHER PARAMS FROM SDF
  /*if (_sdf->HasElement("Motor_Number"))
  {
      Motor_Number = _sdf->Get<double>("Motor_Number");
  }
  else
  {
    ignerr << "Failed to get any <Motor_Number>." << std::endl;
  }*/ 
  if (_sdf->HasElement("Visual_Slowdown"))
  {
      Rot_Vel_Slowdown = _sdf->Get<double>("Visual_Slowdown");
  }
  else
  {
    ignerr << "Failed to get any <Visual_Slowdown>." << std::endl;
  }
  
  //std::cout << "Rot_Vel_Slowdown " << Rot_Vel_Slowdown << std::endl;

  // COLLECT MOTOR TURNING DIRECTION
  if (!_sdf->HasElement("Turning_Direction"))
  {
      ignerr << "[ignition_motor_model] 'Turning_Direction' not specified" << std::endl;
      return;
  }

  std::string turningDirection = _sdf->Get<std::string>("Turning_Direction");
  if (turningDirection == "ccw")
  {
      this->Turn_Direction = 1;
  }
  else if (turningDirection == "cw")
  {
      this->Turn_Direction = -1;
  }
  else
  {
      ignerr << "[ignition_motor_model] Please only use 'cw' or 'ccw' as Turning_Direction" << std::endl;
  }

  if (_sdf->HasElement("initial_velocity"))
  {
    this->dataPtr->jointVelCmd = _sdf->Get<double>("initial_velocity");
    ignmsg << "Joint velocity initialized to ["
           << this->dataPtr->jointVelCmd << "]" << std::endl;
  }

  /*
  // Subscribe to commands
  // When sub topic x topic x Actuator x
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
  // When subtopic x topic x Actuator o
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
  }*/


}

void Motor_Plugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                         ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("Motor_Plugin::PreUpdate");
  
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


  this->dataPtr->Sampling_time = std::chrono::duration_cast<std::chrono::microseconds>(_info.simTime).count()-this->dataPtr->Prev_Sim_time;
  this->dataPtr->Prev_Sim_time = std::chrono::duration_cast<std::chrono::microseconds>(_info.simTime).count();
  //this->dataPtr->UpdateForcesAndMoments();
  
  // SET VISUAL VELOCTIY OF ROTOR
  this->Rot_Vel = sqrt(dataPtr->Thrust/Thrust_Coeff); 
  this->dataPtr->jointVelCmd = Turn_Direction * Rot_Vel / Rot_Vel_Slowdown;


}

void Motor_Plugin::Update(const ignition::gazebo::UpdateInfo &_info,
                         ignition::gazebo::EntityComponentManager &_ecm)
{
 
}

void Motor_Plugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                            const ignition::gazebo::EntityComponentManager &_ecm)
{

}

void Motor_PluginPrivate::UpdateForcesAndMoments()
{
  // UPDATE THRUST VIA FIRST ORDER FILTER (INSTANTANEOUS THRUSTS ARE NOT POSSIBLE)
  if (Thrust_input >= Prev_Thrust)
  {
    // x(k) = alpha*x(k-1) + (1-alpha)*u(k)
    double alpha_up = exp(-Sampling_time/Tau_up);
    Thrust = alpha_up*Prev_Thrust + (1-alpha_up)*Thrust_input;
  }
  else // Thrust_input < prev_thrust
  {
    // x(k) = alpha*x(k-1) + (1-alpha)*u(k)
    double alpha_down = exp(-Sampling_time/Tau_down);
    Thrust = alpha_down*Prev_Thrust + (1-alpha_down)*Thrust_input;
  }

  
    /*
    // APPLY ROTOR THRUST TO LINK
    Link_Ptr->AddRelativeForce(ignition::math::Vector3d(0, 0, (Thrust*g2Newton)));

    // APPLY ROTOR TORQUE TO MAIN BODY
    Torque = Torque_Coeff*(Thrust*g2Newton);
    ignition::math::Vector3d Torque_Vec(0, 0, -Turn_Direction * Torque); // Torque is opposite direction of rotation

    physics::Link_V Parent_Links = Link_Ptr->GetParentJointsLinks(); // Get <vector> of parent links
    ignition::math::Pose3d Pose_Difference = Link_Ptr->WorldCoGPose() - Parent_Links.at(0)->WorldCoGPose(); // Find rotor pos relative to body
    ignition::math::Vector3d Torque_Parent_Frame = Pose_Difference.Rot().RotateVector(Torque_Vec); // Rotate Torque vector to match body orientation
    Parent_Links.at(0)->AddRelativeTorque(Torque_Parent_Frame); // Apply Torque vector to body


    Prev_Thrust = Thrust;

    Thrust_msg.Motor_Number = Motor_Number;
    Thrust_msg.MotorThrust = Thrust_input;
    Thrust_msg.MotorThrust_actual = Thrust;
    MS_Data_Pub.publish(Thrust_msg);
    */

}

IGNITION_ADD_PLUGIN(Motor_Plugin,
                    ignition::gazebo::System,
                    Motor_Plugin::ISystemConfigure,
                    Motor_Plugin::ISystemPreUpdate,
                    Motor_Plugin::ISystemUpdate,
                    Motor_Plugin::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(Motor_Plugin, "ignition::gazebo::systems::Motor_Plugin")
