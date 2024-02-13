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
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    public: JointController();

    public: ~JointController() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

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
