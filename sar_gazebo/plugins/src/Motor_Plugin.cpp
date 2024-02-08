#include "Motor_Plugin.h"

//Configure called once when the simulation starts or when the plugin is first loaded.
void Motor_Plugin::Configure(const ignition::gazebo::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         ignition::gazebo::EntityComponentManager &_ecm,
                         ignition::gazebo::EventManager &_eventMgr)
{
    // Check plugin is running
    std::cout << "Loading Motor_Plugin!!!\n";
    
    // if remove cannot find joint
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
    std::cout << "\t Joint Entity Name:\t" << jointEntity << std::endl;


    // GRAB ROTOR LINK FROM SDF
    if (!_sdf->HasElement("Link_Name")){
        ignerr << "Missing required parameter <Link_Name>" << std::endl;
        return;}

    std::string Prop_Link_Name = _sdf->Get<std::string>("Link_Name");
    std::cout << "\t Link Name:\t" << Prop_Link_Name << std::endl;

    // Find the Link entity by name
    auto linkEntity = this->model.LinkByName(_ecm, Prop_Link_Name);
    if (linkEntity == ignition::gazebo::kNullEntity){
        ignerr << "Link '" << Prop_Link_Name << "' not found." << std::endl;
        return;}
    std::cout << "\t Link Entity Name:\t" << linkEntity << std::endl;


    // COLLECT OTHER PARAMS FROM SDF
    int Motor_Number = _sdf->Get<int>("Motor_Number");
    double Rot_Vel_Slowdown = _sdf->Get<double>("Visual_Slowdown");

    // COLLECT MOTOR TURNING DIRECTION
    if (_sdf->Get<std::string>("Turning_Direction") == "ccw"){
        Turn_Direction = 1;
    }else if(_sdf->Get<std::string>("Turning_Direction") == "cw"){
        Turn_Direction = -1;
    }else{
        ignerr << "[gazebo_motor_model] Please only use 'cw' or 'ccw' as Turning_Direction" << std::endl;
    };



    // Create a JointVelocityCmd component if it doesn't exist    
    if (!_ecm.EntityHasComponentType(jointEntity, ignition::gazebo::components::JointVelocityCmd().TypeId()))
    {
        _ecm.CreateComponent(jointEntity, ignition::gazebo::components::JointVelocityCmd());
    }

    // Set the force or torque on the joint using the correct component
    //auto& jointVelocityCmd = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(jointEntity);
    auto jointVelocityCmd = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(jointEntity);
    std::cout << "\t jointVelocityCmd!!!!!!!!:\t" << jointVelocityCmd << std::endl;
    jointVelocityCmd->Data().resize(1);
    std::cout << "\t jointVelocityCmd!!!!!!!!:\t" << jointVelocityCmd->Data().size() << std::endl;
    
    if (jointVelocityCmd && jointVelocityCmd->Data().size() > 0){
        jointVelocityCmd->Data()[0] = Turn_Direction * 100;
        std::cout << "\t jointVelocityCmd@@\t" << jointVelocityCmd->Data()[0] << std::endl;
    }
    else
    {
        
    }


    
    /*// Set visual velocity of rotor
    
    Rot_Vel = sqrt(1);
    auto jointVelocityCmd = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(jointEntity);
    std::cout << "\t jointVelocityCmd!!!!!!!!:\t" << jointVelocityCmd << std::endl;
    
    if (jointVelocityCmd)
    {
        jointVelocityCmd->Data()[0] = Turn_Direction * 10;
    }*/

}

void Motor_Plugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                         ignition::gazebo::EntityComponentManager &_ecm)
{


    (void)_info; // Explicitly mark _info as unused
    (void)_ecm;  // Explicitly mark _ecm as unused
}


void Motor_Plugin::Update(const ignition::gazebo::UpdateInfo &_info, 
                        ignition::gazebo::EntityComponentManager &_ecm)
{
    
    
    
    /*if (_info.paused)
        return;

    double currentSimTime = std::chrono::duration<double>(_info.simTime).count();
    // Calculate the time elapsed since the last update
    double timeStep = currentSimTime - Prev_Sim_time;
    Prev_Sim_time = currentSimTime;
    // Your logic to update forces and moments
    //UpdateForcesAndMoments();
    std::cout << "\t Joint Name:\t" << Motor_Joint_Name << std::endl;
    std::cout << "\t Joint Entity Name:\t" << jointEntity << std::endl;


    // Set visual velocity of rotor
    double Rot_Vel = sqrt(1);

    auto jointVelocityCmd = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(jointEntity);
    
    if (jointVelocityCmd)
    {
        jointVelocityCmd->Data()[0] = Turn_Direction * Rot_Vel / Rot_Vel_Slowdown;    
    }*/
}



IGNITION_ADD_PLUGIN(Motor_Plugin,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemPreUpdate,
                    ignition::gazebo::ISystemUpdate)
                    
//IGNITION_ADD_PLUGIN_ALIAS(Motor_Plugin, "my_namespace::Motor_Plugin")