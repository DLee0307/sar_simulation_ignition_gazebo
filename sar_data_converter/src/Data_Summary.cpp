#include "SAR_DataConverter.h"

void SAR_DataConverter::Publish_StateData()
{

    // ===================
    //     FLIGHT DATA
    // ===================
    rclcpp::Clock::SharedPtr clock = this->get_clock();
    rclcpp::Duration Time_delta = clock->now() - Time_start;
    //rclcpp::Duration Time_delta = Time - Time_start;
    StateData_msg.time.sec = Time_delta.seconds();
    StateData_msg.time.nanosec = Time_delta.nanoseconds();

    //ros::Duration Time_delta(Time-Time_start);
    //StateData_msg.Time.data.sec = Time_delta.sec;
    //StateData_msg.Time.data.nsec = Time_delta.nsec;

    // STATES WRT ORIGIN
    StateData_msg.pose_b_o = Pose_B_O;
    StateData_msg.twist_b_o = Twist_B_O;
    StateData_msg.accel_b_o = Accel_B_O;
    StateData_msg.eul_b_o = Eul_B_O;
    StateData_msg.accel_b_o_mag = Accel_B_O_Mag;

    // STATES WRT PLANE
    StateData_msg.pose_p_b = Pose_P_B;
    StateData_msg.twist_b_p = Twist_B_P;
    StateData_msg.eul_p_b = Eul_P_B;
    StateData_msg.vel_mag_b_p = Vel_mag_B_P;
    StateData_msg.vel_angle_b_p = Vel_angle_B_P;
    StateData_msg.d_perp = D_perp;
    StateData_msg.d_perp_cr = D_perp_CR;
    StateData_msg.d_perp_pad = D_perp_pad;
    StateData_msg.d_perp_pad_min = D_perp_pad_min;


    // OPTICAL FLOW STATES
    StateData_msg.optical_flow = Optical_Flow;
    StateData_msg.optical_flow_cam = Optical_Flow_Cam;
    StateData_msg.tau_cr = Tau_CR;

    // STATE SETPOINTS
    StateData_msg.x_d = x_d;
    StateData_msg.v_d = v_d;
    StateData_msg.a_d = a_d;

    // CONTROL ACTIONS
    StateData_msg.fm = FM;
    StateData_msg.motorthrusts = MotorThrusts;
    StateData_msg.motor_cmd = Motor_CMD;

    // POLICY ACTIONS
    //StateData_msg.NN_Output = NN_Output;
    StateData_msg.a_trg = a_Trg;
    StateData_msg.a_rot = a_Rot;


    // PUBLISH STATE DATA RECEIVED FROM CONTROLLER
    StateData_Pub->publish(StateData_msg);
    //std::cout << "StateData is published: " << std::endl; 
}

void SAR_DataConverter::Publish_TriggerData()
{
    //ros::Duration Time_delta(Time_trg-Time_start);
    //TriggerData_msg.Time_trg.data.sec = Time_delta.sec;
    //TriggerData_msg.Time_trg.data.nsec = Time_delta.nsec;

    // STATES WRT ORIGIN
    TriggerData_msg.pose_b_o_trg = Pose_B_O_trg;
    TriggerData_msg.twist_b_o_trg = Twist_B_O_trg;
    TriggerData_msg.eul_b_o_trg = Eul_B_O_trg;

    TriggerData_msg.vel_mag_b_o_trg = Vel_mag_B_O_trg;
    TriggerData_msg.vel_angle_b_o_trg = Vel_angle_B_O_trg;

    // STATES WRT PLANE
    TriggerData_msg.pose_p_b_trg = Pose_P_B_trg;
    TriggerData_msg.twist_b_p_trg = Twist_B_P_trg;
    TriggerData_msg.eul_p_b_trg = Eul_P_B_trg;

    TriggerData_msg.vel_mag_b_p_trg = Vel_mag_B_P_trg;
    TriggerData_msg.vel_angle_b_p_trg = Vel_angle_B_P_trg;
    TriggerData_msg.d_perp_trg = D_perp_trg;
    TriggerData_msg.d_perp_cr_trg = D_perp_CR_trg;

    // OPTICAL FLOW STATES
    TriggerData_msg.optical_flow_trg = Optical_Flow_trg;
    TriggerData_msg.tau_cr_trg = Tau_CR_trg;

    // POLICY ACTIONS
    //TriggerData_msg.nn_output_trg = NN_Output_trg;
    TriggerData_msg.a_trg_trg = a_Trg_trg;
    TriggerData_msg.a_rot_trg = a_Rot_trg;

    TriggerData_Pub->publish(TriggerData_msg);

}