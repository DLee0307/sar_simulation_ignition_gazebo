//#include <ignition/physics/Physics.hh> error can solve using below webpage?
//https://gazebosim.org/api/gazebo/5.0/src_2systems_2physics_2Physics_8hh.html

#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/plugin/RegisterMore.hh>

#include <ignition/transport/Node.hh>
#include <ignition/common/Time.hh>
#include <sdf/sdf.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>

#include <vector>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Serialization.hh>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/components/JointPosition.hh>

#include "DiffDrive.hh"
#include <ignition/msgs/odometry.pb.h>
#include <limits>
#include <mutex>
#include <set>
#include <string>
#include <vector>
#include <ignition/common/Profiler.hh>
#include <ignition/math/DiffDriveOdometry.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include "ignition/gazebo/components/CanonicalLink.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/PoseCmd.hh"
#include "ignition/gazebo/components/JointVelocityCmd.hh"
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"
#include "SpeedLimiter.hh"


// CUSTOM INCLUDES
//#include "sar_msgs/CTRL_Data.h"
//#include "sar_msgs/MS.h"
//#include <sar_msgs/msg/ctrl_data.hpp>
//#include <sar_msgs/msg/ms.hpp>

#define g2Newton (9.81f/1000.0f)
#define Newton2g (1000.0f/9.81f)

struct Commands
{
  /// \brief Linear velocity.
  double lin;
  /// \brief Angular velocity.
  double ang;
  Commands() : lin(0.0), ang(0.0) {}
};
class MyJointControllerPluginPrivate; 

class MyJointControllerPlugin : public ignition::gazebo::System,
                                public ignition::gazebo::ISystemConfigure,
                                public ignition::gazebo::ISystemPreUpdate,
                                public ignition::gazebo::ISystemUpdate,
                                public ignition::gazebo::ISystemPostUpdate
{
public:
    // Constructor
    MyJointControllerPlugin();

    // Destructor
    virtual ~MyJointControllerPlugin() = default;


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

    // PostUpdate method
    void PostUpdate(const ignition::gazebo::UpdateInfo &_info, 
                const ignition::gazebo::EntityComponentManager &_ecm) override;

    

private:            
    std::unique_ptr<MyJointControllerPluginPrivate> dataPtr;

};

class MyJointControllerPluginPrivate {
public: 
    void OnCmdVel(const ignition::msgs::Twist &_msg);
    void OnCmdPose(const ignition::msgs::Pose &_msg);
    void UpdateOdometry(const ignition::gazebo::UpdateInfo &_info, const ignition::gazebo::EntityComponentManager &_ecm);
    void UpdateVelocity(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm);
    void UpdatePose(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm);
    
    ignition::transport::Node node;


    std::vector<ignition::gazebo::Entity> leftJoints;
    std::vector<ignition::gazebo::Entity> rightJoints;
    std::vector<ignition::gazebo::Entity> models;
    std::vector<std::string> modelNames;
    std::vector<std::string> leftJointNames;
    std::vector<std::string> rightJointNames;
    double leftJointSpeed{0};
    double rightJointSpeed{0};
    ignition::math::Pose3d modelPose;
    double wheelSeparation{1.0};
    double wheelRadius{0.2};
    ignition::gazebo::Model model{ignition::gazebo::kNullEntity};
    ignition::gazebo::Link canonicalLink{ignition::gazebo::kNullEntity};
    std::chrono::steady_clock::duration odomPubPeriod{0};
    std::chrono::steady_clock::duration lastOdomPubTime{0};
    ignition::math::DiffDriveOdometry odom;
    ignition::transport::Node::Publisher odomPub;
    ignition::transport::Node::Publisher tfPub;
    std::unique_ptr<ignition::gazebo::systems::SpeedLimiter> limiterLin;
    std::unique_ptr<ignition::gazebo::systems::SpeedLimiter> limiterAng;
    Commands last0Cmd;
    Commands last1Cmd;
    ignition::msgs::Twist targetVel;
    ignition::msgs::Pose targetPose;
    std::mutex mutex;
    std::string sdfFrameId;
    std::string sdfChildFrameId;

};
