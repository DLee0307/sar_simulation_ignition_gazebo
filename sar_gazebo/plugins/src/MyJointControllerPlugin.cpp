#include "MyJointControllerPlugin.h"

//using namespace ignition;
//using namespace gazebo;
//using namespace systems;

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
 
}




void MyJointControllerPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                         ignition::gazebo::EntityComponentManager &_ecm)
{

}


void MyJointControllerPlugin::Update(const ignition::gazebo::UpdateInfo &_info, 
                        ignition::gazebo::EntityComponentManager &_ecm)
{
}

void MyJointControllerPlugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info, 
                                        const ignition::gazebo::EntityComponentManager &_ecm)
{

}

void MyJointControllerPluginPrivate::UpdateOdometry(const ignition::gazebo::UpdateInfo &_info,
                                                    const ignition::gazebo::EntityComponentManager &_ecm)
{
 
}

void MyJointControllerPluginPrivate::UpdateVelocity(const ignition::gazebo::UpdateInfo &_info,
                                                    const ignition::gazebo::EntityComponentManager &_ecm)
{
 
}

void MyJointControllerPluginPrivate::UpdatePose(const ignition::gazebo::UpdateInfo &_info,
                                                const ignition::gazebo::EntityComponentManager &/*_ecm*/)
{


}


void MyJointControllerPluginPrivate::OnCmdVel(const ignition::msgs::Twist &_msg)
{

}


void MyJointControllerPluginPrivate::OnCmdPose(const ignition::msgs::Pose &_msg)
{

}

IGNITION_ADD_PLUGIN(MyJointControllerPlugin,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemPreUpdate,
                    ignition::gazebo::ISystemUpdate,
                    ignition::gazebo::ISystemPostUpdate)
                    
IGNITION_ADD_PLUGIN_ALIAS(MyJointControllerPlugin, "my_namespace::MyJointControllerPlugin")