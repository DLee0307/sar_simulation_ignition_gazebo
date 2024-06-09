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
/*
    // STATES WRT PLANE
    Pose_P_B = ctrl_msg.Pose_P_B;
    Pose_P_B.orientation.x = NAN; // Quaternion is not used
    Pose_P_B.orientation.y = NAN;
    Pose_P_B.orientation.z = NAN;
    Pose_P_B.orientation.w = NAN;

    Twist_B_P = ctrl_msg.Twist_B_P;
    Vel_mag_B_P = ctrl_msg.Vel_mag_B_P;
    Vel_angle_B_P = ctrl_msg.Vel_angle_B_P;

    Eul_P_B.x = NAN;
    Eul_P_B.y = Plane_Angle_deg - Eul_B_O.y;
    Eul_P_B.z = NAN;
    

    // STATES RELATIVE TO LANDING SURFACE
    D_perp = ctrl_msg.D_perp;
    D_perp_CR = ctrl_msg.D_perp_CR;


    float Beta1_deg = -Eul_P_B.y - Gamma_eff + 90;
    float Beta1_rad = Beta1_deg*M_PI/180;

    float Beta2_deg = Gamma_eff - Eul_P_B.y + 90;
    float Beta2_rad = Beta2_deg*M_PI/180;

    geometry_msgs::Vector3 r_B_O;
    r_B_O.x = Pose_B_O.position.x;
    r_B_O.y = Pose_B_O.position.y;
    r_B_O.z = Pose_B_O.position.z;

    
    Eigen::Vector3d r_C1_B(L_eff,0,0);
    Eigen::Vector3d r_C2_B(L_eff,0,0);
    Eigen::Vector3d r_P_B(Pose_P_B.position.x,Pose_P_B.position.y,Pose_P_B.position.z); // {t_x,t_y,n_p}
    Eigen::Vector3d r_B_P = -r_P_B; // {t_x,t_y,n_p}

    Eigen::Matrix3d R_C1P;

    R_C1P << cos(Beta1_rad), 0, sin(Beta1_rad),
             0, 1, 0,
             -sin(Beta1_rad), 0, cos(Beta1_rad);

    Eigen::Matrix3d R_C2P;

    R_C2P << cos(Beta2_rad), 0, sin(Beta2_rad),
             0, 1, 0,
             -sin(Beta2_rad), 0, cos(Beta2_rad);

    r_C1_B = R_C1P*r_C1_B; // {t_x,t_y,n_p}
    r_C2_B = R_C2P*r_C2_B; // {t_x,t_y,n_p}

    Eigen::Vector3d r_C1_P = r_B_P + r_C1_B; // {t_x,t_y,n_p}
    Eigen::Vector3d r_C2_P = r_B_P + r_C2_B; // {t_x,t_y,n_p}

    D_perp_pad = std::min(abs(r_C1_P(2)),abs(r_C2_P(2)));
    if (D_perp_pad < D_perp_pad_min)
    {
        D_perp_pad_min = D_perp_pad;
    }

    // LANDING SURFACE STATES
    Plane_Pos = ctrl_msg.Plane_Pos;
    Plane_Angle_deg = ctrl_msg.Plane_Angle_deg;

    // OPTICAL FLOW STATES
    Optical_Flow = ctrl_msg.Optical_Flow;
    Optical_Flow_Cam = ctrl_msg.Optical_Flow_Cam;
    
    Theta_x = Optical_Flow.x;
    Theta_y = Optical_Flow.y;
    Tau = Optical_Flow.z;
    Tau_CR = ctrl_msg.Tau_CR;

    // ESTIMATED OPTICAL FLOW STATES
    Theta_x_Cam = Optical_Flow_Cam.x;
    Theta_y_Cam = Optical_Flow_Cam.y;    
    Tau_Cam = Optical_Flow_Cam.z;

    // STATE SETPOINTS
    x_d = ctrl_msg.x_d;
    v_d = ctrl_msg.v_d;
    a_d = ctrl_msg.a_d;

    // CONTROL ACTIONS
    FM = ctrl_msg.FM;
    FM[0] = FM[0]*Newton2g;
    FM[1] = FM[1]*Newton2g*1.0e-3;
    FM[2] = FM[2]*Newton2g*1.0e-3;
    FM[3] = FM[3]*Newton2g*1.0e-3;
    MotorThrusts = ctrl_msg.MotorThrusts;
    Motor_CMD = ctrl_msg.Motor_CMD;


    // NEURAL NETWORK DATA
    NN_Output = ctrl_msg.NN_Output;
    a_Trg = ctrl_msg.a_Trg;
    a_Rot = ctrl_msg.a_Rot;

    Pose_B_O_impact_buff.push_back(Pose_B_O);
    Eul_B_O_impact_buff.push_back(Eul_B_O);

    Twist_P_B_impact_buff.push_back(Twist_B_P);
    Eul_P_B_impact_buff.push_back(Eul_P_B);


    // =================
    //   TRIGGER DATA
    // =================

    Trg_Flag = ctrl_msg.Trg_Flag;
    if(ctrl_msg.Trg_Flag == true && OnceFlag_Trg == false)
    {   
        Time_trg = ros::Time::now();
        OnceFlag_Trg = true;
        Rot_Sum = Eul_B_O.y;

    }

    if(ctrl_msg.Trg_Flag == true)
    {
        double Time_delta = Time.toSec()-Time_prev.toSec();
        Rot_Sum += (Time_delta*Twist_B_O.angular.y)*180/M_PI;
        // printf("Val: %f\n",Rot_Sum);
    }
    

    
    // STATES WRT ORIGIN
    Pose_B_O_trg = ctrl_msg.Pose_B_O_trg;
    Pose_B_O_trg.orientation.x = NAN; // Quaternion is not used
    Pose_B_O_trg.orientation.y = NAN;
    Pose_B_O_trg.orientation.z = NAN;
    Pose_B_O_trg.orientation.w = NAN;
    Twist_B_O_trg = ctrl_msg.Twist_B_O_trg;

    Vel_mag_B_O_trg = sqrt(pow(Twist_B_O_trg.linear.x,2)+pow(Twist_B_O_trg.linear.z,2));
    Vel_angle_B_O_trg = atan2(Twist_B_O_trg.linear.z,Twist_B_O_trg.linear.x)*180/M_PI;

    float quat_trg[4] = {
        (float)ctrl_msg.Pose_B_O_trg.orientation.x,
        (float)ctrl_msg.Pose_B_O_trg.orientation.y,
        (float)ctrl_msg.Pose_B_O_trg.orientation.z,
        (float)ctrl_msg.Pose_B_O_trg.orientation.w
    };
    float eul_trg[3];
    quat2euler(quat_trg,eul_trg);
    Eul_B_O_trg.x = eul_trg[0]*180/M_PI;
    Eul_B_O_trg.y = eul_trg[1]*180/M_PI;
    Eul_B_O_trg.z = eul_trg[2]*180/M_PI;

    // STATES WRT PLANE
    Pose_P_B_trg = ctrl_msg.Pose_P_B_trg;
    Pose_P_B_trg.orientation.x = NAN; // Quaternion is not used
    Pose_P_B_trg.orientation.y = NAN;
    Pose_P_B_trg.orientation.z = NAN;
    Pose_P_B_trg.orientation.w = NAN;
    Twist_B_P_trg = ctrl_msg.Twist_B_P_trg;

    Eul_P_B_trg.x = NAN;
    Eul_P_B_trg.y = Plane_Angle_deg - Eul_B_O_trg.y;
    Eul_P_B_trg.z = NAN;

    Vel_mag_B_P_trg = sqrt(pow(Twist_B_P_trg.linear.x,2)+pow(Twist_B_P_trg.linear.z,2));;
    Vel_angle_B_P_trg = atan2(Twist_B_P_trg.linear.z,Twist_B_P_trg.linear.x)*180/M_PI;


    // OPTICAL FLOW
    Optical_Flow_trg = ctrl_msg.Optical_Flow_trg;
    Theta_x_trg = Optical_Flow_trg.x;
    Theta_y_trg = Optical_Flow_trg.y;
    Tau_trg = Optical_Flow_trg.z;
    Tau_CR_trg = ctrl_msg.Tau_CR_trg;
    D_perp_trg = ctrl_msg.Pose_P_B_trg.position.z;
    D_perp_CR_trg = ctrl_msg.D_perp_CR_trg;


    // POLICY ACTION DATA
    NN_Output_trg = ctrl_msg.NN_Output_trg;
    a_Trg_trg = ctrl_msg.a_Trg_trg;
    a_Rot_trg = ctrl_msg.a_Rot_trg;

    // =======================
    //   ONBOARD IMPACT DATA
    // =======================

    Impact_Flag_OB = ctrl_msg.Impact_Flag_OB;

    Vel_mag_B_P_impact_OB = ctrl_msg.Vel_mag_B_P_impact_OB;
    Vel_angle_B_P_impact_OB = ctrl_msg.Vel_angle_B_P_impact_OB;
    Pose_B_O_impact_OB = ctrl_msg.Pose_B_O_impact_OB;

    Twist_B_P_impact_OB = ctrl_msg.Twist_B_P_impact_OB;
    Twist_B_P_impact_OB.linear.x = Vel_mag_B_P_impact_OB*cos(Vel_angle_B_P_impact_OB*M_PI/180);
    Twist_B_P_impact_OB.linear.y = NAN;
    Twist_B_P_impact_OB.linear.z = Vel_mag_B_P_impact_OB*sin(Vel_angle_B_P_impact_OB*M_PI/180);

    float quat_impact[4] = {
        (float)ctrl_msg.Pose_B_O_impact_OB.orientation.x,
        (float)ctrl_msg.Pose_B_O_impact_OB.orientation.y,
        (float)ctrl_msg.Pose_B_O_impact_OB.orientation.z,
        (float)ctrl_msg.Pose_B_O_impact_OB.orientation.w
    };

    // PROCESS EULER ANGLES
    float eul_impact[3];
    quat2euler(quat_impact,eul);
    Eul_B_O_impact_OB.x = eul_impact[0]*180/M_PI;
    Eul_B_O_impact_OB.y = eul_impact[1]*180/M_PI;
    Eul_B_O_impact_OB.z = eul_impact[2]*180/M_PI;
    

    Eul_P_B_impact_OB.x = NAN;
    Eul_P_B_impact_OB.y = Plane_Angle_deg - Eul_B_O_impact_OB.y;
    Eul_P_B_impact_OB.z = NAN;


    dOmega_B_O_y_impact_OB = ctrl_msg.dOmega_B_O_y_impact_OB;

    if(ctrl_msg.Impact_Flag_OB == true && OnceFlag_Impact_OB == false)
    {   
        Time_impact_OB = ros::Time::now();
        OnceFlag_Impact_OB = true;

    }

*/


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