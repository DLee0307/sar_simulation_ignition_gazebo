#ifndef IGNITION_GAZEBO_SYSTEMS_Practice_HH_
#define IGNITION_GAZEBO_SYSTEMS_Practice_HH_

#include <iostream>
#include <memory>

// For making plugin INCLUDES  
#include <rclcpp/rclcpp.hpp>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/plugin/RegisterMore.hh>

// From Joint Controller
#include <ignition/msgs/actuators.pb.h>
#include <ignition/msgs/double.pb.h>
#include <string>
#include <vector>
#include <ignition/common/Profiler.hh>
#include <ignition/math/PID.hh>
#include <ignition/transport/Node.hh>
#include "ignition/gazebo/components/Actuators.hh"
#include "ignition/gazebo/components/JointForceCmd.hh"
#include "ignition/gazebo/components/JointVelocity.hh"
#include "ignition/gazebo/components/JointVelocityCmd.hh"
#include "ignition/gazebo/Model.hh"

// From Bryan
#include <ignition/transport/Node.hh>
#include <ignition/common/Time.hh>
// CUSTOM INCLUDES
//#include "sar_msgs/CTRL_Data.h"
//#include "sar_msgs/MS.h"
//#include <sar_msgs/msg/ctrl_data.hpp>
//#include <sar_msgs/msg/ms.hpp>

// Rest
#include <sdf/sdf.hh>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Serialization.hh>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#define g2Newton (9.81f/1000.0f)
#define Newton2g (1000.0f/9.81f)

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
   class PracticePrivate;

   class IGNITION_GAZEBO_VISIBLE Practice
      : public ignition::gazebo::System,
        public ignition::gazebo::ISystemConfigure,
        public ignition::gazebo::ISystemPreUpdate,
        public ignition::gazebo::ISystemUpdate,
        public ignition::gazebo::ISystemPostUpdate
  {
    public: Practice();

    public: ~Practice() override = default;

    // Documentation inherited
    public: void Configure(const ignition::gazebo::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &_ecm,
                           ignition::gazebo::EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: void Update(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) override;

    private: std::unique_ptr<PracticePrivate> dataPtr;
  };
  }
}
}
}

#endif
