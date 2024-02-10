#include "MyJointControllerPlugin.h"


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
    
    std::cout << "\t Link Entity Name:\t" << ptr << std::endl;
}




void MyJointControllerPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                         ignition::gazebo::EntityComponentManager &_ecm)
{
      
}


void MyJointControllerPlugin::Update(const ignition::gazebo::UpdateInfo &_info, 
                        ignition::gazebo::EntityComponentManager &_ecm)
{
    
    
    

}



IGNITION_ADD_PLUGIN(MyJointControllerPlugin,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemPreUpdate,
                    ignition::gazebo::ISystemUpdate)
                    
IGNITION_ADD_PLUGIN_ALIAS(MyJointControllerPlugin, "my_namespace::MyJointControllerPlugin")