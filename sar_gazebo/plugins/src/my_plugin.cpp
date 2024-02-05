#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <sdf/sdf.hh>

namespace ignition
{
namespace gazebo
{
  class MyPlugin : public System,
                   public ISystemConfigure,
                   public ISystemPreUpdate
  {
  public: 
    void Configure(const Entity& entity,
                   const std::shared_ptr<const sdf::Element>& sdf,
                   EntityComponentManager& ecm,
                   EventManager&) override
    {
      model = Model(entity);

      if (!model.Valid(ecm))
      {
        ignerr << "MyPlugin should be attached to a valid model entity." << std::endl;
        return;
      }

      std::string modelName = model.Name(ecm);
      igndbg << "My Ignition Gazebo plugin is attached to model [" << modelName << "]" << std::endl;
    }

    void PreUpdate(const UpdateInfo&, EntityComponentManager&) override
    {
      // Update logic goes here
    }

  private:
    Model model;
  };
}
}

IGNITION_ADD_PLUGIN(ignition::gazebo::MyPlugin,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemPreUpdate)


// Register the plugin with the simulator
IGNITION_ADD_PLUGIN_ALIAS(ignition::gazebo::MyPlugin, "MyPlugin")