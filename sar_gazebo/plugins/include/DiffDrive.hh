#ifndef IGNITION_GAZEBO_SYSTEMS_DIFFDRIVE_HH_
#define IGNITION_GAZEBO_SYSTEMS_DIFFDRIVE_HH_

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
   class DiffDrivePrivate;

   class IGNITION_GAZEBO_VISIBLE DiffDrive
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate,
        public ISystemPostUpdate
  {
    public: DiffDrive();

    public: ~DiffDrive() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;
    // Documentation inherited
    public: void PostUpdate(
                const UpdateInfo &_info,
                const EntityComponentManager &_ecm) override;

    private: std::unique_ptr<DiffDrivePrivate> dataPtr;
  };
  }
}
}
}

#endif