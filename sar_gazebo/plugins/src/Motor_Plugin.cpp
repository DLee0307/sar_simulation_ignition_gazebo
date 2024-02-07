#include "Motor_Plugin.h"

//Configure called once when the simulation starts or when the plugin is first loaded.
void Motor_Plugin::Configure(const ignition::gazebo::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         ignition::gazebo::EntityComponentManager &_ecm,
                         ignition::gazebo::EventManager &_eventMgr)
{
    // Check plugin is running
    std::cout << "Loading Motor_Plugin!!!\n";
    
    // need to check
    this->model = ignition::gazebo::Model(_entity);

    // GRAB MOTOR JOINT FROM SDF
    if (!_sdf->HasElement("Joint_Name")){
        ignerr << "Missing required parameter <Joint_Name>" << std::endl;
        return;}

    std::string Motor_Joint_Name = _sdf->Get<std::string>("Joint_Name");
    std::cout << "\t Joint Name:\t" << Motor_Joint_Name << std::endl;

    // Find the joint entity by name
    auto jointEntity = this->model.JointByName(_ecm, Motor_Joint_Name);
    if (jointEntity == ignition::gazebo::kNullEntity){
        ignerr << "Joint '" << Motor_Joint_Name << "' not found." << std::endl;
        return;}




    // GRAB ROTOR LINK FROM SDF
    if (!_sdf->HasElement("Link_Name")){
        ignerr << "Missing required parameter <Link_Name>" << std::endl;
        return;}

    std::string Prop_Link_Name = _sdf->Get<std::string>("Link_Name");
    std::cout << "\t Link Name:\t" << Prop_Link_Name << std::endl;

    // Find the joint entity by name
    auto linkEntity = this->model.LinkByName(_ecm, Prop_Link_Name);
    if (linkEntity == ignition::gazebo::kNullEntity){
        ignerr << "Link '" << Prop_Link_Name << "' not found." << std::endl;
        return;}


    // COLLECT OTHER PARAMS FROM SDF
    int Motor_Number = _sdf->Get<int>("Motor_Number");
    double Rot_Vel_Slowdown = _sdf->Get<double>("Visual_Slowdown");



}

void Motor_Plugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                         ignition::gazebo::EntityComponentManager &_ecm)
{
    //(void)_info; // Explicitly mark _info as unused
    //(void)_ecm;  // Explicitly mark _ecm as unused
}

IGNITION_ADD_PLUGIN(Motor_Plugin,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemPreUpdate)