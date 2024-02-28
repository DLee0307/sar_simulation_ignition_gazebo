#include "Test.h"
#include <mutex>
#include <string>
#include <ignition/msgs/actuators.pb.h>
#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs/Utility.hh>
#include <sdf/sdf.hh>

#include "ignition/gazebo/components/Actuators.hh"
#include "ignition/gazebo/components/ExternalWorldWrenchCmd.hh"
#include "ignition/gazebo/components/JointAxis.hh"
#include "ignition/gazebo/components/JointVelocity.hh"
#include "ignition/gazebo/components/JointVelocityCmd.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/ParentLinkName.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Wind.hh"
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::TestPrivate
{
  public: Entity linkEntity;
  public: std::string linkName;
  public: Model model{kNullEntity};
};

Test::Test()
  : dataPtr(std::make_unique<TestPrivate>())
{
}

void Test::Configure(const ignition::gazebo::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         ignition::gazebo::EntityComponentManager &_ecm,
                         ignition::gazebo::EventManager &_eventMgr)
{

  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "MulticopterMotorModel plugin should be attached to a model "
           << "entity. Failed to initialize." << std::endl;
    return;
  }    
  std::cout << "1: " << _entity << std::endl;

  auto sdfClone = _sdf->Clone();

  if (sdfClone->HasElement("linkName"))
  {
    this->dataPtr->linkName = sdfClone->Get<std::string>("linkName");
  }

  std::cout << "2: " << this->dataPtr->linkName << std::endl;

  if (this->dataPtr->linkName.empty())
  {
    ignerr << "MulticopterMotorModel found an empty linkName parameter. "
           << "Failed to initialize.";
    return;
  }

}

void Test::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                         ignition::gazebo::EntityComponentManager &_ecm)
{
  if (this->dataPtr->linkEntity == kNullEntity)
  {
    this->dataPtr->linkEntity =
        this->dataPtr->model.LinkByName(_ecm, this->dataPtr->linkName);
  }
  
  std::cout << "3: " << this->dataPtr->linkName << std::endl;
  std::cout << "4: " << this->dataPtr->linkEntity << std::endl;
  using Pose = math::Pose3d;
  //using Vector3 = math::Vector3d;
  ignition::math::Vector3d force(1000, 1000, 1000);
  
  Link link(this->dataPtr->linkEntity);
  const auto worldPose = link.WorldPose(_ecm);

  // Apply a force to the link.
  link.AddWorldForce(_ecm, force);
  //link.AddWorldForce(_ecm,
  //                    worldPose->Rot().RotateVector(Vector3(100, 100, 100)));
  
  //std::cout << "4: " << Link << std::endl;
                      
 
}

void Test::Update(const ignition::gazebo::UpdateInfo &_info,
                         ignition::gazebo::EntityComponentManager &_ecm)
{
 
}

void Test::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                            const ignition::gazebo::EntityComponentManager &_ecm)
{

}


IGNITION_ADD_PLUGIN(Test,
                    ignition::gazebo::System,
                    Test::ISystemConfigure,
                    Test::ISystemPreUpdate,
                    Test::ISystemUpdate,
                    Test::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(Test, "ignition::gazebo::systems::Test")
