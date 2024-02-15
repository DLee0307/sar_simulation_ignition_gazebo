#ifndef MY_PLUGIN_H
#define MY_PLUGIN_H

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/plugin/RegisterMore.hh>

class MyPlugin : public ignition::gazebo::System,
                 public ignition::gazebo::ISystemConfigure,
                 public ignition::gazebo::ISystemPreUpdate
{
public:
    // Constructor
    MyPlugin() = default;

    // Destructor
    virtual ~MyPlugin() = default;

    // Configure method
    void Configure(const ignition::gazebo::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   ignition::gazebo::EntityComponentManager &_ecm,
                   ignition::gazebo::EventManager &_eventMgr) override;

    // PreUpdate method
    void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                   ignition::gazebo::EntityComponentManager &_ecm) override;
private:

};

#endif  // MY_PLUGIN_H
