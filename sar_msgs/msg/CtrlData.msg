uint32 tick

# =========================
## CONTROLLER STATES
# =========================
geometry_msgs/Pose pose_b_o
geometry_msgs/Twist twist_b_o
geometry_msgs/Accel accel_b_o
float64 accel_b_o_mag

geometry_msgs/Pose pose_p_b
geometry_msgs/Twist twist_b_p
float64 vel_mag_p_b
float64 vel_angle_p_b
float64 d_perp
float64 d_perp_cr

## PLANE STATES
geometry_msgs/Vector3 plane_pos
float64 plane_angle_deg

## OPTICAL FLOW STATES
geometry_msgs/Vector3 optical_flow
geometry_msgs/Vector3 optical_flow_cam
float64 tau_cr

geometry_msgs/Vector3 x_d
geometry_msgs/Vector3 v_d
geometry_msgs/Vector3 a_d

float64[4] fm
float64[4] motor_thrusts
uint32[4] ms_pwm

float64 policy_trg_action
float64 policy_rot_action


# ==========================
#  STATES AT POLICY TRIGGER
# ==========================
bool trg_flag

geometry_msgs/Pose pose_b_o_trg
geometry_msgs/Twist twist_b_o_trg

geometry_msgs/Pose pose_p_b_trg
geometry_msgs/Twist twist_b_p_trg
float64 vel_mag_b_p_trg
float64 vel_angle_b_p_trg
float64 d_perp_trg
float64 d_perp_cr_trg

geometry_msgs/Vector3 optical_flow_trg
float64 tau_cr_trg
float64 policy_trg_action_trg
float64 policy_rot_action_trg

# =========================
#      STATES AT IMPACT
# =========================

bool impact_flag_ob

geometry_msgs/Pose pose_b_o_impact_ob
geometry_msgs/Twist twist_b_p_impact_ob
float64 accel_b_o_mag_impact_ob
