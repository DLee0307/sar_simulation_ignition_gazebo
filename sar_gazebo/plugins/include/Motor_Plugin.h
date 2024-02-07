#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/plugin/RegisterMore.hh>
//#include <ignition/physics/Physics.hh> error can solve using below webpage?
//https://gazebosim.org/api/gazebo/5.0/src_2systems_2physics_2Physics_8hh.html
#include <ignition/transport/Node.hh>
#include <ignition/common/Time.hh>

class Motor_Plugin : public ignition::gazebo::System,
                     public ignition::gazebo::ISystemConfigure,
                     public ignition::gazebo::ISystemPreUpdate
{
public:
    // Constructor
    Motor_Plugin() = default;

    // Destructor
    virtual ~Motor_Plugin() = default;

    // Configure method
    void Configure(const ignition::gazebo::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   ignition::gazebo::EntityComponentManager &_ecm,
                   ignition::gazebo::EventManager &_eventMgr) override;

    // PreUpdate method
    void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                   ignition::gazebo::EntityComponentManager &_ecm) override;
private:
    bool messagePrinted = false;
};
