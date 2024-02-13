#include "JointController.hh"
#include <iostream>
#include <ignition/plugin/Register.hh>

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::JointControllerPrivate
{
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

}

void JointController::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                         ignition::gazebo::EntityComponentManager &_ecm)
{

}

IGNITION_ADD_PLUGIN(JointController,
                    ignition::gazebo::System,
                    JointController::ISystemConfigure,
                    JointController::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(JointController, "ignition::gazebo::systems::JointController")