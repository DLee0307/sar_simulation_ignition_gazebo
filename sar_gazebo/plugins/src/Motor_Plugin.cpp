#include "Motor_Plugin.h"

//Configure called once when the simulation starts or when the plugin is first loaded.
void Motor_Plugin::Configure(const ignition::gazebo::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         ignition::gazebo::EntityComponentManager &_ecm,
                         ignition::gazebo::EventManager &_eventMgr)
{
    // Check plugin is running
    std::cout << "Loading Motor_Plugin!!!\n";
    
    // need to check
    this->model = ignition::gazebo::Model(_entity);

    if (!_sdf->HasElement("Joint_Name")){
        ignerr << "Missing required parameter <Joint_Name>" << std::endl;
        return;}

    std::string Motor_Joint_Name = _sdf->Get<std::string>("Joint_Name");
    std::cout << "\t Joint Name:\t" << Motor_Joint_Name << std::endl;

    // Find the joint entity by name
    auto jointEntity = this->model.JointByName(_ecm, Motor_Joint_Name);
    if (jointEntity == ignition::gazebo::kNullEntity){
        ignerr << "Joint '" << Motor_Joint_Name << "' not found." << std::endl;
        return;}
}

void Motor_Plugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                         ignition::gazebo::EntityComponentManager &_ecm)
{
    //(void)_info; // Explicitly mark _info as unused
    //(void)_ecm;  // Explicitly mark _ecm as unused
}

IGNITION_ADD_PLUGIN(Motor_Plugin,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemPreUpdate)