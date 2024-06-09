#include "SAR_DataConverter.h"

void SAR_DataConverter::CtrlData_Callback(const sar_msgs::msg::CtrlData::SharedPtr ctrl_msg) {

    // ===================
    //     FLIGHT DATA
    // ===================
    Time_prev = Time;
    rclcpp::Clock::SharedPtr clock = this->get_clock();
    Time = clock->now();

    // STATES WRT ORIGIN
    Pose_B_O = ctrl_msg->pose_b_o;
    Pose_B_O.orientation.x = NAN; // Quaternion is not used
    Pose_B_O.orientation.y = NAN;
    Pose_B_O.orientation.z = NAN;
    Pose_B_O.orientation.w = NAN;
    Twist_B_O = ctrl_msg->twist_b_o;
    Accel_B_O = ctrl_msg->accel_b_o;
    Accel_B_O_Mag = ctrl_msg->accel_b_o_mag;

    // PROCESS EULER ANGLES
    float quat[4] = {
        (float)ctrl_msg->pose_b_o.orientation.x,
        (float)ctrl_msg->pose_b_o.orientation.y,
        (float)ctrl_msg->pose_b_o.orientation.z,
        (float)ctrl_msg->pose_b_o.orientation.w
    };
    float eul[3];
    quat2euler(quat,eul);
    Eul_B_O.x = eul[0]*180/M_PI;
    Eul_B_O.y = eul[1]*180/M_PI;
    Eul_B_O.z = eul[2]*180/M_PI;

    Vel_mag_B_O = sqrt(pow(Twist_B_O.linear.x,2)+pow(Twist_B_O.linear.z,2));
    Vel_angle_B_O = atan2(Twist_B_O.linear.z,Twist_B_O.linear.x)*180/M_PI;

    float quat_trg[4] = {
        (float)ctrl_msg->pose_b_o_trg.orientation.x,
        (float)ctrl_msg->pose_b_o_trg.orientation.y,
        (float)ctrl_msg->pose_b_o_trg.orientation.z,
        (float)ctrl_msg->pose_b_o_trg.orientation.w
    };
    float eul_trg[3];
    quat2euler(quat_trg,eul_trg);
    Eul_B_O_trg.x = eul_trg[0]*180/M_PI;
    Eul_B_O_trg.y = eul_trg[1]*180/M_PI;
    Eul_B_O_trg.z = eul_trg[2]*180/M_PI;

    // STATES WRT PLANE
    Pose_P_B_trg = ctrl_msg->pose_p_b_trg;
    Pose_P_B_trg.orientation.x = NAN; // Quaternion is not used
    Pose_P_B_trg.orientation.y = NAN;
    Pose_P_B_trg.orientation.z = NAN;
    Pose_P_B_trg.orientation.w = NAN;
    Twist_B_P_trg = ctrl_msg->twist_b_p_trg;

    Eul_P_B_trg.x = NAN;
    Eul_P_B_trg.y = Plane_Angle_deg - Eul_B_O_trg.y;
    Eul_P_B_trg.z = NAN;

    Vel_mag_B_P_trg = ctrl_msg->vel_mag_b_p_trg;
    Vel_angle_B_P_trg = ctrl_msg->vel_angle_b_p_trg;

    // OPTICAL FLOW
    Optical_Flow_trg = ctrl_msg->optical_flow_trg;
    Theta_x_trg = Optical_Flow_trg.x;
    Theta_y_trg = Optical_Flow_trg.y;
    Tau_trg = Optical_Flow_trg.z;
    Tau_CR_trg = ctrl_msg->tau_cr_trg;
    D_perp_trg = ctrl_msg->pose_p_b_trg.position.z;
    D_perp_CR_trg = ctrl_msg->d_perp_cr_trg;

    // POLICY ACTION DATA
    Policy_Trg_Action_trg = ctrl_msg->policy_trg_action_trg;
    Policy_Rot_Action_trg = ctrl_msg->policy_rot_action_trg;

    // =======================
    //   ONBOARD IMPACT DATA
    // =======================

    Impact_Flag_OB = ctrl_msg->impact_flag_ob;

    if(ctrl_msg->impact_flag_ob == true && OnceFlag_Impact_OB == false)
    {   
        Time_impact_OB = clock->now(); 
        OnceFlag_Impact_OB = true;

    }

    Pose_B_O_impact_OB = ctrl_msg->pose_b_o_impact_ob;
    Pose_B_O_impact_OB.orientation.x = NAN; // Quaternion is not used
    Pose_B_O_impact_OB.orientation.y = NAN;
    Pose_B_O_impact_OB.orientation.z = NAN;
    Pose_B_O_impact_OB.orientation.w = NAN;
    Twist_B_P_impact_OB = ctrl_msg->twist_b_p_impact_ob;
    Accel_B_O_Mag_impact_OB = ctrl_msg->accel_b_o_mag_impact_ob;

}

void SAR_DataConverter::CtrlDebug_Callback(const sar_msgs::msg::CtrlDebug::SharedPtr ctrl_msg) 
{
    Tumbled_Flag = ctrl_msg->tumbled_flag;
    TumbleDetect_Flag = ctrl_msg->tumbledetect_flag;
    MotorStop_Flag = ctrl_msg->motorstop_flag;
    AngAccel_Flag = ctrl_msg->angaccel_flag;
    Armed_Flag = ctrl_msg->armed_flag;
    CustomThrust_Flag = ctrl_msg->customthrust_flag;
    CustomMotorCMD_Flag = ctrl_msg->custommotorcmd_flag;
    
    Pos_Ctrl_Flag = ctrl_msg->pos_ctrl_flag;
    Vel_Ctrl_Flag = ctrl_msg->vel_ctrl_flag;
    Policy_Armed_Flag = ctrl_msg->policy_armed_flag;
    CamActive_Flag = ctrl_msg->camactive_flag;
}