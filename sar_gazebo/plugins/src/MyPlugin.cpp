#include "MyPlugin.h"
#include <iostream>

void MyPlugin::Configure(const ignition::gazebo::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         ignition::gazebo::EntityComponentManager &_ecm,
                         ignition::gazebo::EventManager &_eventMgr)
{
    std::cout << "Configuring MyPlugin for entity: " << _entity << std::endl;

    // Example: Read a parameter from the SDF file
    if (_sdf->HasElement("example_param")) {
        auto exampleParam = _sdf->Get<std::string>("example_param");
        std::cout << "Example parameter: " << exampleParam << std::endl;
    } else {
        std::cout << "No example_param found in SDF." << std::endl;
    }
}

void MyPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                         ignition::gazebo::EntityComponentManager &_ecm)
{
    std::cout << "PreUpdate called at simulation time: " 
              << _info.simTime.count() << std::endl;
}

IGNITION_ADD_PLUGIN(MyPlugin,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemPreUpdate)