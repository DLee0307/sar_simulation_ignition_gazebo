#ifndef IGNITION_GAZEBO_SYSTEMS_JOINTCONTROLLER_HH_
#define IGNITION_GAZEBO_SYSTEMS_JOINTCONTROLLER_HH_

#include <memory>
#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
   class JointControllerPrivate;

   class IGNITION_GAZEBO_VISIBLE JointController
      : public ignition::gazebo::System,
        public ignition::gazebo::ISystemConfigure,
        public ignition::gazebo::ISystemPreUpdate
  {
    public: JointController();

    public: ~JointController() override = default;

    // Documentation inherited
    public: void Configure(const ignition::gazebo::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &_ecm,
                           ignition::gazebo::EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;


    private: std::unique_ptr<JointControllerPrivate> dataPtr;
  };
  }
}
}
}

#endif
