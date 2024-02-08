#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/plugin/RegisterMore.hh>
//#include <ignition/physics/Physics.hh> error can solve using below webpage?
//https://gazebosim.org/api/gazebo/5.0/src_2systems_2physics_2Physics_8hh.html
#include <ignition/transport/Node.hh>
#include <ignition/common/Time.hh>
#include <sdf/sdf.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>

// CUSTOM INCLUDES
//#include "sar_msgs/CTRL_Data.h"
//#include "sar_msgs/MS.h"
//#include <sar_msgs/msg/ctrl_data.hpp>
//#include <sar_msgs/msg/ms.hpp>

#define g2Newton (9.81f/1000.0f)
#define Newton2g (1000.0f/9.81f)

class Motor_Plugin : public ignition::gazebo::System,
                     public ignition::gazebo::ISystemConfigure,
                     public ignition::gazebo::ISystemPreUpdate,
                     public ignition::gazebo::ISystemUpdate
{
public:
    // Constructor
    Motor_Plugin();

    // Destructor
    virtual ~Motor_Plugin() = default;

    // Configure method
    void Configure(const ignition::gazebo::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   ignition::gazebo::EntityComponentManager &_ecm,
                   ignition::gazebo::EventManager &_eventMgr) override;

    // PreUpdate method
    void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                   ignition::gazebo::EntityComponentManager &_ecm) override;
    
    // Update method
    void Update(const ignition::gazebo::UpdateInfo &_info, 
                ignition::gazebo::EntityComponentManager &_ecm) override;
                

    

private:            
            // if remove we cannot find joint
            ignition::gazebo::Model model;

            //DH's editing make node
            rclcpp::Node::SharedPtr node_;
            
            
            // CONFIG PARAMS
            std::string SAR_Type;
            std::string SAR_Config;
            std::string Cam_Config;

            // GAZEBO POINTERS
            ignition::gazebo::Entity worldEntity;    // physics::WorldPtr
            ignition::gazebo::Entity modelEntity;    // physics::ModelPtr
            ignition::gazebo::Entity jointEntity;    // physics::JointPtr
            ignition::gazebo::Entity linkEntity;     // physics::LinkPtr

            // MOTOR PARAMETERS
            std::string Motor_Joint_Name;
            std::string Prop_Link_Name;

            int Motor_Number;
            int Turn_Direction;

            double Thrust_Coeff;    // Thrust Coeff [N/rad/s]
            double Torque_Coeff;    // Torque Coeff [N*m/rad/s]
            double C_tf;        // Torque-Thrust Coeff [N*m/N]

            // FIRST ORDER FILTER BEHAVIOR
            float Thrust_input = 0.0f;  // Desired Thrust [N]
            double Tau_up;              // Motor Time Constant (Up) [s]
            double Tau_down;            // Motor Time Constant (Down) [s]
            double Sampling_time;
            double Prev_Sim_time = 0.0;
            double Prev_Thrust = 0.0;
            
            // CACULATED VALUES
            double Thrust;              // Calculated Thrust [N]
            double Torque;              // Calculated Torque [N*m]
            double Rot_Vel = 0.0f;      // Rotational Velocity [rad/s]
            double Rot_Vel_Slowdown;    // Slowed-down Rotational Velocity [rad/s]
    

            // ROS CONNECTIONS
            
            //ros::NodeHandle nh;
            //make node using 

            //ros::Subscriber CTRL_Data_Sub = nh.subscribe<sar_msgs::CTRL_Data>("/CTRL/data", 1, &Motor_Plugin::CtrlData_Callback, this, ros::TransportHints().tcpNoDelay());
            //ros::Publisher MS_Data_Pub = nh.advertise<sar_msgs::MS>("/SAR_Internal/MS",1);
            //sar_msgs::MS Thrust_msg;
};

Motor_Plugin::Motor_Plugin() : Prev_Sim_time(0.0)
{
}
