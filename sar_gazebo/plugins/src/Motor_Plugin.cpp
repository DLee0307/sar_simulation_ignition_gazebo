#include "Motor_Plugin.h"

//Configure called once when the simulation starts or when the plugin is first loaded.
void Motor_Plugin::Configure(const ignition::gazebo::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         ignition::gazebo::EntityComponentManager &_ecm,
                         ignition::gazebo::EventManager &_eventMgr)
{
    std::cout << "Loading Motor_Plugin!!!!!!\n";

}
void Motor_Plugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                         ignition::gazebo::EntityComponentManager &_ecm)
{

}

IGNITION_ADD_PLUGIN(Motor_Plugin,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemPreUpdate)