#include "MyPlugin.h"

void MyPlugin::Configure(const ignition::gazebo::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         ignition::gazebo::EntityComponentManager &_ecm,
                         ignition::gazebo::EventManager &_eventMgr)

{
    std::cout << "MyPlugin::Configure called for entity: " << _entity << std::endl;
}

void MyPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                         ignition::gazebo::EntityComponentManager &_ecm)

{
    // For simplification, no operation is performed here
}

IGNITION_ADD_PLUGIN(MyPlugin,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemPreUpdate)