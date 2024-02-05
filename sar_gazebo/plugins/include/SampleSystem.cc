#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/System.hh>

using namespace ignition;
using namespace gazebo;

class SampleSystem : public System, public ISystemConfigure
{
public:
  void Configure(const Entity &_entity, 
                 const std::shared_ptr<const sdf::Element> &_sdf, 
                 EntityComponentManager &_ecm, 
                 EventManager &_eventMgr) override
  {
    std::cout << "SampleSystem plugin loaded!" << std::endl;
  }
};

// Register this plugin with the simulator
IGNITION_ADD_PLUGIN(SampleSystem, System, ISystemConfigure)
