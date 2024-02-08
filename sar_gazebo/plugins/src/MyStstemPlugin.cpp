#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/System.hh>


class MySystemPlugin : public ignition::gazebo::System,
                       public ignition::gazebo::ISystemUpdate
{
public:
  MySystemPlugin() {}
;
  void Update(const ignition::gazebo::UpdateInfo &_info,
              ignition::gazebo::EntityComponentManager &_ecm) override
  {
    std::cout << "Update event triggered!" << std::endl;
  }
};

IGNITION_ADD_PLUGIN(MySystemPlugin,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemUpdate)
IGNITION_ADD_PLUGIN_ALIAS(MySystemPlugin, "my_namespace::MySystemPlugin")