#include "MyJointControllerPlugin.h"



//using namespace ignition;
//using namespace gazebo;
//using namespace systems;

//https://gist.github.com/nullpo24/146f253f0d789cb10c680786e7582940
MyJointControllerPlugin::MyJointControllerPlugin()
    : dataPtr(std::make_unique<MyJointControllerPluginPrivate>())
{
}

//Configure called once when the simulation starts or when the plugin is first loaded.
void MyJointControllerPlugin::Configure(const ignition::gazebo::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         ignition::gazebo::EntityComponentManager &_ecm,
                         ignition::gazebo::EventManager &_eventMgr)
{
    this->dataPtr->model = ignition::gazebo::Model(_entity);

    // Get the canonical link
    std::vector<ignition::gazebo::Entity> links = _ecm.ChildrenByComponents(
        this->dataPtr->model.Entity(), ignition::gazebo::components::CanonicalLink());
    
    if (!links.empty())
        this->dataPtr->canonicalLink = ignition::gazebo::Link(links[0]);
    
    if (!this->dataPtr->model.Valid(_ecm))
    {
        ignerr << "DiffDrive plugin should be attached to a model entity. "
            << "Failed to initialize." << std::endl;
        return;
    }
    auto ptr = const_cast<sdf::Element *>(_sdf.get());

    // Get params from SDF
    sdf::ElementPtr sdfElem = ptr->GetElement("left_joint");
    while (sdfElem)
    {
        this->dataPtr->leftJointNames.push_back(sdfElem->Get<std::string>());
        sdfElem = sdfElem->GetNextElement("left_joint");
    }

    sdfElem = ptr->GetElement("right_joint");
    while (sdfElem)
    {
        this->dataPtr->rightJointNames.push_back(sdfElem->Get<std::string>());
        sdfElem = sdfElem->GetNextElement("right_joint");
    }

    //can be problem in this part
    sdfElem = ptr->GetElement("model");
    while (sdfElem)
    {
        this->dataPtr->modelNames.push_back(sdfElem->Get<std::string>());
        sdfElem = sdfElem->GetNextElement("model");
        std::cout << "\t Link Entity Name:!!!\t" << ptr << std::endl;
    }

    this->dataPtr->wheelSeparation = _sdf->Get<double>("wheel_separation",
        this->dataPtr->wheelSeparation).first;
    this->dataPtr->wheelRadius = _sdf->Get<double>("wheel_radius",
        this->dataPtr->wheelRadius).first;

    // Parse speed limiter parameters.
    bool hasVelocityLimits     = false;
    bool hasAccelerationLimits = false;
    bool hasJerkLimits         = false;
    double minVel              = std::numeric_limits<double>::lowest();
    double maxVel              = std::numeric_limits<double>::max();
    double minAccel            = std::numeric_limits<double>::lowest();
    double maxAccel            = std::numeric_limits<double>::max();
    double minJerk             = std::numeric_limits<double>::lowest();
    double maxJerk             = std::numeric_limits<double>::max();

    if (_sdf->HasElement("min_velocity"))
    {
        minVel = _sdf->Get<double>("min_velocity");
        hasVelocityLimits = true;
    }
    if (_sdf->HasElement("max_velocity"))
    {
        maxVel = _sdf->Get<double>("max_velocity");
        hasVelocityLimits = true;
    }
    if (_sdf->HasElement("min_acceleration"))
    {
        minAccel = _sdf->Get<double>("min_acceleration");
        hasAccelerationLimits = true;
    }
    if (_sdf->HasElement("max_acceleration"))
    {
        maxAccel = _sdf->Get<double>("max_acceleration");
        hasAccelerationLimits = true;
    }
    if (_sdf->HasElement("min_jerk"))
    {
        minJerk = _sdf->Get<double>("min_jerk");
        hasJerkLimits = true;
    }
    if (_sdf->HasElement("max_jerk"))
    {
        maxJerk = _sdf->Get<double>("max_jerk");
        hasJerkLimits = true;
    }
    
    ignition::math::SpeedLimiter speedLimiter;
    if (hasVelocityLimits) {
        speedLimiter.SetMinVelocity(minVel);
        speedLimiter.SetMaxVelocity(maxVel);
    }
    if (hasAccelerationLimits) {
        speedLimiter.SetMinAcceleration(minAccel);
        speedLimiter.SetMaxAcceleration(maxAccel);
    }
    if (hasJerkLimits) {
        speedLimiter.SetMinJerk(minJerk);
        speedLimiter.SetMaxJerk(maxJerk);
    }
    
    /*this->dataPtr->limiterLin = std::make_unique<SpeedLimiter>(
        hasVelocityLimits, hasAccelerationLimits, hasJerkLimits,
        minVel, maxVel, minAccel, maxAccel, minJerk, maxJerk);
    
    this->dataPtr->limiterAng = std::make_unique<SpeedLimiter>(
        hasVelocityLimits, hasAccelerationLimits, hasJerkLimits,
        minVel, maxVel, minAccel, maxAccel, minJerk, maxJerk);*/
    
    double odomFreq = _sdf->Get<double>("odom_publish_frequency", 50).first;
    if (odomFreq > 0)
    {
        std::chrono::duration<double> odomPer{1 / odomFreq};
        this->dataPtr->odomPubPeriod =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(odomPer);
    }


    // Setup odometry.
    this->dataPtr->odom.SetWheelParams(this->dataPtr->wheelSeparation,
        this->dataPtr->wheelRadius, this->dataPtr->wheelRadius);

    // Subscribe to commands
    std::string topic{"/model/" + this->dataPtr->model.Name(_ecm) + "/cmd_vel"};
    if (_sdf->HasElement("topic"))
        topic = _sdf->Get<std::string>("topic");
    std::cout << "topic:" << topic << std::endl;
    this->dataPtr->node.Subscribe(topic, &MyJointControllerPluginPrivate::OnCmdVel,
        this->dataPtr.get());

    std::string odomTopic{"/model/" + this->dataPtr->model.Name(_ecm) +
        "/odometry"};
    this->dataPtr->odomPub = this->dataPtr->node.Advertise<ignition::msgs::Odometry>(
        odomTopic);


    // Subscribe to pose commands
    // this->dataPtr->node.Subscribe("/PosePublisher/geometry_msgs/Pose", &DiffDrivePrivate::OnCmdPose, this->dataPtr.get());
    this->dataPtr->node.Subscribe("/pose_publisher", &MyJointControllerPluginPrivate::OnCmdPose, this->dataPtr.get());

    if (_sdf->HasElement("frame_id"))
        this->dataPtr->sdfFrameId = _sdf->Get<std::string>("frame_id");

    if (_sdf->HasElement("child_frame_id"))
        this->dataPtr->sdfChildFrameId = _sdf->Get<std::string>("child_frame_id");

    ignmsg << "DiffDrive subscribing to twist messages on [" << topic << "]"
            << std::endl;

    std::cout << "\t Link Entity Name:\t" << ptr << std::endl;
}




void MyJointControllerPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                         ignition::gazebo::EntityComponentManager &_ecm)
{
    IGN_PROFILE("DiffDrive::PreUpdate");

    // \TODO(anyone) Support rewind
    if (_info.dt < std::chrono::steady_clock::duration::zero())
    {
        ignwarn << "Detected jump back in time ["
            << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
            << "s]. System may not work properly." << std::endl;
    }

    // If the joints haven't been identified yet, look for them
    static std::set<std::string> warnedModels;
    auto modelName = this->dataPtr->model.Name(_ecm);
    if (this->dataPtr->leftJoints.empty() ||
        this->dataPtr->rightJoints.empty() ||
        this->dataPtr->models.empty())
    {
        bool warned{false};
        for (const std::string &name : this->dataPtr->leftJointNames)
        {
            ignition::gazebo::Entity joint = this->dataPtr->model.JointByName(_ecm, name);
            if (joint != ignition::gazebo::kNullEntity)
                this->dataPtr->leftJoints.push_back(joint);
            else if (warnedModels.find(modelName) == warnedModels.end())
            {
                ignwarn << "Failed to find left joint [" << name << "] for model ["
                        << modelName << "]" << std::endl;
                warned = true;
            }
        }
        
        for (const std::string &name : this->dataPtr->rightJointNames)
        {
            ignition::gazebo::Entity joint = this->dataPtr->model.JointByName(_ecm, name);
            if (joint != ignition::gazebo::kNullEntity)
                this->dataPtr->rightJoints.push_back(joint);
            else if (warnedModels.find(modelName) == warnedModels.end())
            {
                ignwarn << "Failed to find right joint [" << name << "] for model ["
                        << modelName << "]" << std::endl;
                warned = true;
            }
        }
        for (const std::string &name : this->dataPtr->modelNames)
        {
            ignition::gazebo::Entity joint = this->dataPtr->model.JointByName(_ecm, name);
            if (joint != ignition::gazebo::kNullEntity)
                this->dataPtr->models.push_back(joint);
        }

        if (warned)
        {
            warnedModels.insert(modelName);
        }

    }
    
    if (this->dataPtr->leftJoints.empty() || this->dataPtr->rightJoints.empty())
        return;


    if (warnedModels.find(modelName) != warnedModels.end())
    {
        ignmsg << "Found joints for model [" << modelName
            << "], plugin will start working." << std::endl;
        warnedModels.erase(modelName);
    }


    // Nothing left to do if paused.
    if (_info.paused)
        return;

    for (ignition::gazebo::Entity joint : this->dataPtr->leftJoints)
    {
        // Update wheel velocity
        auto vel = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(joint);

        if (vel == nullptr)
        {
        _ecm.CreateComponent(
            joint, ignition::gazebo::components::JointVelocityCmd({this->dataPtr->leftJointSpeed}));
        }
        else
        {
        *vel = ignition::gazebo::components::JointVelocityCmd({this->dataPtr->leftJointSpeed});
        }
    }

    for (ignition::gazebo::Entity joint : this->dataPtr->rightJoints)
    {
        // Update wheel velocity
        auto vel = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(joint);

        if (vel == nullptr)
        {
        _ecm.CreateComponent(joint,
            ignition::gazebo::components::JointVelocityCmd({this->dataPtr->rightJointSpeed}));
        }
        else
        {
        *vel = ignition::gazebo::components::JointVelocityCmd({this->dataPtr->rightJointSpeed});
        }
    }

    // Create the left and right side joint position components if they
    // don't exist.
    auto leftPos = _ecm.Component<ignition::gazebo::components::JointPosition>(
        this->dataPtr->leftJoints[0]);
    if (!leftPos)
    {
        _ecm.CreateComponent(this->dataPtr->leftJoints[0],
            ignition::gazebo::components::JointPosition());
    }

    auto rightPos = _ecm.Component<ignition::gazebo::components::JointPosition>(
        this->dataPtr->rightJoints[0]);
    if (!rightPos)
    {
        _ecm.CreateComponent(this->dataPtr->rightJoints[0],
            ignition::gazebo::components::JointPosition());
    }

    auto pose = ignition::gazebo::convert<ignition::math::Pose3d>(this->dataPtr->targetPose);
    this->dataPtr->model.SetWorldPoseCmd(_ecm, pose);
}


void MyJointControllerPlugin::Update(const ignition::gazebo::UpdateInfo &_info, 
                        ignition::gazebo::EntityComponentManager &_ecm)
{
    
    
    

}




void MyJointControllerPluginPrivate::OnCmdVel(const ignition::msgs::Twist &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->targetVel = _msg;
}


void MyJointControllerPluginPrivate::OnCmdPose(const ignition::msgs::Pose &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->targetPose = _msg;
}

IGNITION_ADD_PLUGIN(MyJointControllerPlugin,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemPreUpdate,
                    ignition::gazebo::ISystemUpdate)
                    
IGNITION_ADD_PLUGIN_ALIAS(MyJointControllerPlugin, "my_namespace::MyJointControllerPlugin")