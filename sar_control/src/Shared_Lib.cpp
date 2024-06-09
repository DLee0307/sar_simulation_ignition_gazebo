#include "Shared_Lib.h"


// =================================
//    INITIAL SYSTEM PARAMETERS
// =================================
float m = 0.0f;             // [kg]
float Ixx = 0.0f;           // [kg*m^2]
float Iyy = 0.0f;           // [kg*m^2]
float Izz = 0.0f;           // [kg*m^2]
struct mat33 J;             // Rotational Inertia Matrix [kg*m^2]

float C_tf = 0.0f;          // Moment Coeff [Nm/N]
float Thrust_max = 0.0f;         // Max thrust per motor [g]

const float g = 9.81f;                        // Gravity [m/s^2]
const struct vec e_3 = {0.0f, 0.0f, 1.0f};    // Global z-axis

float dt = (float)(1.0f/RATE_100_HZ);
uint32_t prev_tick = 0;
struct CTRL_CmdPacket CTRL_Cmd;

// =================================
//    ROS2 PARAMETER
// =================================
float Gamma_eff = 0.0f; 
float K_Pitch = 0.0f; 
float K_Yaw = 0.0f;

std::string Plane_Type;
std::string Plane_Config;

std::string POLICY_TYPE;

int Vicon_Delay_ms = 0;

float SIM_SPEED = 0.5;
float SIM_SLOWDOWN_SPEED = 0.5;
bool LANDING_SLOWDOWN_FLAG = true;

int LOGGING_RATE = 10;
bool SHOW_CONSOLE = true;

float Leg_Length = 0.0;
int Leg_Angle = 0;

float Tau_up = 0.0;
float Tau_down= 0.0;
std::vector<double> TrajAcc_Max = {0.0f, 0.0f, 0.0f};
std::vector<double> TrajJerk_Max = {0.0f, 0.0f, 0.0f};

float Base_Mass = 0.0f;             // [kg]
float Base_Ixx = 0.0f;           // [kg*m^2]
float Base_Iyy = 0.0f;           // [kg*m^2]
float Base_Izz = 0.0f;           // [kg*m^2]

// =================================
//       GEOMETRIC PARAMETERS
// =================================

float Prop_14_x = 0.0f;         // Front Prop Distance - x-axis [m]
float Prop_14_y = 0.0f;         // Front Prop Distance - y-axis [m]
float Prop_23_x = 0.0f;         // Rear  Prop Distance - x-axis [m]
float Prop_23_y = 0.0f;         // Rear  Prop Distance - y-axis [m]

float L_eff = 0.0f;             // Effective Leg Length [m]
float Forward_Reach = 0.0f;     // Forward Reach [m]
float Collision_Radius = 0.0f;  // Collision Radius [m]


// =================================
//    CONTROL GAIN INITIALIZATION
// =================================
// XY POSITION PID
float P_kp_xy = 0.0f;
float P_kd_xy = 0.0f;
float P_ki_xy = 0.0f;
float i_range_xy = 0.0f;

// Z POSITION PID
float P_kp_z = 0.0f;
float P_kd_z = 0.0f;
float P_ki_z = 0.0f;
float i_range_z = 0.0f;

// XY ATTITUDE PID
float R_kp_xy = 0.0f;
float R_kd_xy = 0.0f;
float R_ki_xy = 0.0f;
float i_range_R_xy = 0.0f;

// Z ATTITUDE PID
float R_kp_z = 0.0f;
float R_kd_z = 0.0f;
float R_ki_z = 0.0f;
float i_range_R_z = 0.0f;


// INIT CTRL GAIN VECTORS 
struct vec Kp_p;    // Pos. Proportional Gains 
struct vec Kd_p;    // Pos. Derivative Gains
struct vec Ki_p;    // Pos. Integral Gains  

struct vec Kp_R;    // Rot. Proportional Gains
struct vec Kd_R;    // Rot. Derivative Gains
struct vec Ki_R;    // Rot. Integral Gains


// INIT GAIN FLAGS
float kp_xf = 1;    // Pos. Gain Flag
float kd_xf = 1;    // Pos. Derivative Gain Flag


// =================================
//     BODY WRT ORIGIN STATES
// =================================
struct vec Pos_B_O = {0.0f,0.0f,0.0f};          // Pos [m]
struct vec Vel_B_O = {0.0f,0.0f,0.0f};          // Vel [m/s]
struct vec Accel_B_O = {0.0f,0.0f,0.0f};        // Linear Accel. [m/s^2]
float Accel_B_O_Mag = 0.0f;                     // Linear Acceleration Magnitude [m/s^2]

struct quat Quat_B_O = {0.0f,0.0f,0.0f,1.0f};   // Orientation
struct vec Omega_B_O = {0.0f,0.0f,0.0f};        // Angular Rate [rad/s]
struct vec Omega_B_O_prev  = {0.0f,0.0f,0.0f};  // Prev Angular Rate [rad/s^2]
struct vec dOmega_B_O = {0.0f,0.0f,0.0f};       // Angular Accel [rad/s^2]

struct mat33 R;                                 // Orientation as rotation matrix
struct vec b3 = {0.0f,0.0f,1.0f};               // Current body z-axis in global coord.

// =================================
//     BODY WRT PLANE STATES
// =================================
struct vec Pos_P_B = {0.0f,0.0f,0.0f};          // Pos [m]
struct vec Vel_B_P = {0.0f,0.0f,0.0f};          // Vel [m/s]
struct quat Quat_P_B = {0.0f,0.0f,0.0f,1.0f};   // Orientation
struct vec Omega_B_P = {0.0f,0.0f,0.0f};        // Angular Rate [rad/s]

// RELATIVE STATES
float D_perp = 0.0f;                            // Distance perp to plane [m]
float D_perp_CR = 0.0f;                         // Distance from CR to plane [m]
float Vel_mag_B_P = 0.0f;                       // Velocity magnitude relative [m/s]
float Vel_angle_B_P = 0.0f;                     // Velocity angle relative [deg]
                            

// =================================
//         DESIRED STATES
// =================================
struct vec x_d = {0.0f,0.0f,0.0f};              // Pos-desired [m]
struct vec v_d = {0.0f,0.0f,0.0f};              // Vel-desired [m/s]
struct vec a_d = {0.0f,0.0f,0.0f};              // Acc-desired [m/s^2]

struct vec b1_d = {1.0f,0.0f,0.0f};             // Desired body x-axis in global coord. 
struct vec b2_d = {0.0f,1.0f,0.0f};             // Desired body y-axis in global coord.
struct vec b3_d = {0.0f,0.0f,1.0f};             // Desired body z-axis in global coord.
struct mat33 R_d;                               // Desired rotational matrix from b_d vectors

struct quat quat_d = {0.0f,0.0f,0.0f,1.0f};     // Orientation-desired [qx,qy,qz,qw]
struct vec omega_d = {0.0f,0.0f,0.0f};          // Omega-desired [rad/s]
struct vec domega_d = {0.0f,0.0f,0.0f};         // Ang. Acc-desired [rad/s^2]

// =================================
//         STATE ERRORS
// =================================
struct vec e_x;  // Pos-error [m]
struct vec e_v;  // Vel-error [m/s]
struct vec e_PI; // Pos. Integral-error [m*s]

struct vec e_R;  // Rotation-error [rad]
struct vec e_w;  // Omega-error [rad/s]
struct vec e_RI; // Rot. Integral-error [rad*s]


// =================================
//       CONTROLLER ACTUATIONS
// =================================
struct vec F_thrust_ideal = {0.0f,0.0f,1.0f};   // Ideal thrust vector [N]
float F_thrust = 0.0f;                          // Implemented body thrust [N]
struct vec M = {0.0f,0.0f,0.0f};                // Implemented body moments [N*m]
struct vec M_d = {0.0f,0.0f,0.0f};              // Desired moment [N*m]

// MOTOR THRUST ACTIONS
float f_thrust_g = 0.0f;    // Motor thrust - Thrust [g]
float f_roll_g = 0.0f;      // Motor thrust - Roll   [g]
float f_pitch_g = 0.0f;     // Motor thrust - Pitch  [g]
float f_yaw_g = 0.0f;       // Motor thrust - Yaw    [g]

// INDIVIDUAL MOTOR THRUSTS
float M1_thrust = 0.0f;     // Motor 1 [g]
float M2_thrust = 0.0f;     // Motor 2 [g]
float M3_thrust = 0.0f;     // Motor 3 [g]
float M4_thrust = 0.0f;     // Motor 4 [g]

// MOTOR PWM VALUES
uint16_t M1_pwm = 0;        // [0 - 65,535]
uint16_t M2_pwm = 0;        // [0 - 65,535]
uint16_t M3_pwm = 0;        // [0 - 65,535]
uint16_t M4_pwm = 0;        // [0 - 65,535]

// CONTROL OVERRIDE VALUES
uint16_t PWM_override[4] = {0,0,0,0};               // Motor PWM values
float thrust_override[4] = {0.0f,0.0f,0.0f,0.0f};   // Motor thrusts [g] 


// =================================
//   TEMPORARY CALC VECS/MATRICES
// =================================
static struct vec temp1_v; 
static struct vec temp2_v;
static struct vec temp3_v;
static struct vec temp4_v;
static struct mat33 temp1_m;  

static struct vec P_effort; // Effort by positional PID
static struct vec R_effort; // Effort by rotational PID

static struct mat33 RdT_R;  // Rd' * R
static struct mat33 RT_Rd;  // R' * Rd
static struct vec Gyro_dyn;

// =================================
//  FLAGS AND SYSTEM INITIALIZATION
// =================================

// CONTROLLER FLAGS
bool Tumbled_Flag = false;
bool TumbleDetect_Flag = true;
bool MotorStop_Flag = false;
bool AngAccel_Flag = false;
bool Armed_Flag = false;
bool CustomThrust_Flag = false;
bool CustomMotorCMD_Flag = false;
uint16_t CMD_ID = 0;


// SENSOR FLAGS
bool CamActive_Flag = false;

// =================================
//       POLICY INITIALIZATION
// =================================

// DEFINE POLICY TYPE ACTIVATED
PolicyType Policy = PARAM_OPTIM;

// POLICY FLAGS
bool Policy_Armed_Flag = false;
bool Trg_Flag = false;
bool onceFlag = false;

// =================================
//    LANDING SURFACE PARAMETERS
// =================================
float Plane_Angle_deg = 0.0f;           // Plane Angle [deg]
struct vec r_P_O = {0.0f,0.0f,0.0f};    // Plane Position Vector        [m]

void CTRL_Command(struct CTRL_CmdPacket *CTRL_Cmd)
{
    switch(CTRL_Cmd->cmd_type){
        case 0: // Reset
            //std::cout << "Case 0:" << std::endl;

            break;

        case 1: // Position
            x_d.x = CTRL_Cmd->cmd_val1;
            x_d.y = CTRL_Cmd->cmd_val2;
            x_d.z = CTRL_Cmd->cmd_val3;
            kp_xf = CTRL_Cmd->cmd_flag;
            break;

        case 2: // Velocity
            v_d.x = CTRL_Cmd->cmd_val1;
            v_d.y = CTRL_Cmd->cmd_val2;
            v_d.z = CTRL_Cmd->cmd_val3;
            kd_xf = CTRL_Cmd->cmd_flag;
            break;

        case 5: // Hard Set All Motorspeeds to Zero
            //std::cout << "Case 5:" << std::endl;

            break;   

        case 10: // Upload Point-to-Point Trajectory Values

            Traj_Type = P2P;
            axis = (axis_direction)CTRL_Cmd->cmd_flag;

            switch(axis){

                case x_axis:

                    s_0_t[0] = Pos_B_O.x;           // Starting position [m]
                    s_f_t[0] = CTRL_Cmd->cmd_val2;  // Ending position [m]
                    a_t[0] = CTRL_Cmd->cmd_val3;    // Peak acceleration [m/s^2]
                    t_traj[0] = 0.0f;               // Reset timer

                    break;

                case y_axis:

                    s_0_t[1] = Pos_B_O.y;           // Starting position [m]
                    s_f_t[1] = CTRL_Cmd->cmd_val2;  // Ending position [m]
                    a_t[1] = CTRL_Cmd->cmd_val3;    // Peak acceleration [m/s^2]
                    t_traj[1] = 0.0f;               // Reset timer

                    break;

                case z_axis:

                    s_0_t[2] = Pos_B_O.z;           // Starting position [m]
                    s_f_t[2] = CTRL_Cmd->cmd_val2;  // Ending position [m]
                    a_t[2] = CTRL_Cmd->cmd_val3;    // Peak acceleration [m/s^2]
                    t_traj[2] = 0.0f;               // Reset timer

                    break;
            }

            break;

        case 11: // Constant Velocity Trajectory
            //std::cout << "Case 11:" << std::endl;

            Traj_Type = CONST_VEL;
            axis = (axis_direction)CTRL_Cmd->cmd_flag;

            switch(axis){

                case x_axis:

                    s_0_t[0] = CTRL_Cmd->cmd_val1;               // Starting position [m]
                    v_t[0] = CTRL_Cmd->cmd_val2;                 // Desired velocity [m/s]
                    a_t[0] = CTRL_Cmd->cmd_val3;                 // Acceleration [m/s^2]

                    t_traj[0] = 0.0f; // Reset timer
                    break;

                case y_axis:

                    s_0_t[1] = CTRL_Cmd->cmd_val1;
                    v_t[1] = CTRL_Cmd->cmd_val2;
                    a_t[1] = CTRL_Cmd->cmd_val3;

                    t_traj[1] = 0.0f;
                    break;

                case z_axis:

                    s_0_t[2] = CTRL_Cmd->cmd_val1;
                    v_t[2] = CTRL_Cmd->cmd_val2;
                    a_t[2] = CTRL_Cmd->cmd_val3;

                    t_traj[2] = 0.0f;
                    break;
                    
            }

            break;    

        case 19: // ACTIVATE TRAJECTORY

            Traj_Active[0] = (bool)CTRL_Cmd->cmd_val1;
            Traj_Active[1] = (bool)CTRL_Cmd->cmd_val2;
            Traj_Active[2] = (bool)CTRL_Cmd->cmd_val3;

            break;

    }
}

void controlOutput(const state_t *state, const sensorData_t *sensors)
{
    J = mdiag(Ixx,Iyy,Izz);// In controllerOutOfTreeReset()
    //printmat(J);

    // CONTROL GAINS
    Kp_p = mkvec(P_kp_xy,P_kp_xy,P_kp_z);
    //printvec(Kp_p);
    Kd_p = mkvec(P_kd_xy,P_kd_xy,P_kd_z);
    //printvec(Kd_p);
    Ki_p = mkvec(P_ki_xy,P_ki_xy,P_ki_z);
    //printvec(Ki_p);

    Kp_R = mkvec(R_kp_xy,R_kp_xy,R_kp_z);
    Kd_R = mkvec(R_kd_xy,R_kd_xy,R_kd_z);
    Ki_R = mkvec(R_ki_xy,R_ki_xy,R_ki_z);
    
    // =========== STATE SETPOINTS =========== //
    omega_d = mkvec(0.0f,0.0f,0.0f);        // Omega-desired [rad/s]
    domega_d = mkvec(0.0f,0.0f,0.0f);       // Omega-Accl. [rad/s^2]
    quat_d = mkquat(0.0f,0.0f,0.0f,1.0f);   // Desired orientation 

    // =========== ROTATION MATRIX =========== //
    // R changes Body axes to be in terms of Global axes
    // https://www.andre-gaschler.com/rotationconverter/
    //printquat(Quat_B_O);
    R = quat2rotmat(Quat_B_O); // Quaternion to Rotation Matrix Conversion
    //printmat(R);
    b3 = mvmul(R, e_3);         // Current body vertical axis in terms of global axes | [b3 = R*e_3] 
        


    // =========== TRANSLATIONAL EFFORT =========== //
    e_x = vsub(Pos_B_O, x_d); // [e_x = pos-x_d]
    //printvec(Pos_B_O);
    //printvec(x_d);
    //printvec(e_x);
    e_v = vsub(Vel_B_O, v_d); // [e_v = vel-v_d]

    // POS. INTEGRAL ERROR
    e_PI.x += (e_x.x)*dt;
    e_PI.x = clamp(e_PI.x, -i_range_xy, i_range_xy);

    e_PI.y += (e_x.y)*dt;
    e_PI.y = clamp(e_PI.y, -i_range_xy, i_range_xy);

    e_PI.z += (e_x.z)*dt;
    e_PI.z = clamp(e_PI.z, -i_range_z, i_range_z);

    /* [F_thrust_ideal = -kp_x*e_x*(kp_x_flag) + -kd_x*e_v + -kI_x*e_PI*(kp_x_flag) + m*g*e_3 + m*a_d] */
    /*
    math3d.h : https://github.com/jpreiss/cmath3d/blob/master/math3d.h
    clamp = (A min max) if A < min, then min. if A > max, then max
    veltmul = element-wise vector multiply.
    vneg = -1 * vector
    vscl = scalar * vector
    vadd3 = Adding three vector
    mvmul = multiply a matrix by a vector.
    mmul = multiply two matrice.
    dehat = make from matrix to vector
    */
    temp1_v = veltmul(vneg(Kp_p), e_x);
    temp1_v = vscl(kp_xf,temp1_v);
    temp2_v = veltmul(vneg(Kd_p), e_v);
    temp1_v = vscl(kd_xf,temp1_v); // typo?
    temp3_v = veltmul(vneg(Ki_p), e_PI);
    P_effort = vadd3(temp1_v,temp2_v,temp3_v);
    temp1_v = vscl(m*g, e_3); // Feed-forward term
    temp2_v = vscl(m, a_d);

    F_thrust_ideal = vadd3(P_effort, temp1_v, temp2_v); 

    // =========== DESIRED BODY AXES =========== // 
    b3_d = vnormalize(F_thrust_ideal);
    b2_d = vnormalize(vcross(b3_d, b1_d));      // [b3_d x b1_d] | body-fixed horizontal axis
    temp1_v = vnormalize(vcross(b2_d, b3_d));
    R_d = mcolumns(temp1_v, b2_d, b3_d);        // Desired rotation matrix from calculations

    // =========== ROTATIONAL ERRORS =========== // 
    RdT_R = mmul(mtranspose(R_d), R);       // [R_d'*R]
    RT_Rd = mmul(mtranspose(R), R_d);       // [R'*R_d]

    temp1_v = dehat(msub(RdT_R, RT_Rd));    // [dehat(R_d'*R - R'*R)]
    e_R = vscl(0.5f, temp1_v);              // Rotation error | [eR = 0.5*dehat(R_d'*R - R'*R)]


    // Rotational error=1/2​(trace(RdT​R)−1)=1/2((aA+dD+gG)+(bB+eE+hH)+(cC+fF+iI)−1)

    temp1_v = mvmul(RT_Rd, omega_d);        // [R'*R_d*omega_d]
    e_w = vsub(Omega_B_O, temp1_v);        // Ang. vel error | [e_w = omega - R'*R_d*omega_d] 

    // ROT. INTEGRAL ERROR
    e_RI.x += (e_R.x)*dt;
    e_RI.x = clamp(e_RI.x, -i_range_R_xy, i_range_R_xy);

    e_RI.y += (e_R.y)*dt;
    e_RI.y = clamp(e_RI.y, -i_range_R_xy, i_range_R_xy);

    e_RI.z += (e_R.z)*dt;
    e_RI.z = clamp(e_RI.z, -i_range_R_z, i_range_R_z);

    // =========== CONTROL EQUATIONS =========== // 
    /* [M = -kp_R*e_R - kd_R*e_w -ki_R*e_RI + Gyro_dyn] */

    temp1_v = veltmul(vneg(Kp_R), e_R);     // [-kp_R*e_R]
    temp2_v = veltmul(vneg(Kd_R), e_w);     // [-kd_R*e_w]
    temp3_v = veltmul(vneg(Ki_R), e_RI);    // [-ki_R*e_RI]
    R_effort = vadd3(temp1_v,temp2_v,temp3_v);

    /* Gyro_dyn = [omega x (J*omega)] - [J*( hat(omega)*R'*R_d*omega_d - R'*R_d*domega_d )] */
    temp1_v = vcross(Omega_B_O, mvmul(J, Omega_B_O)); // [omega x J*omega]


    temp1_m = mmul(hat(Omega_B_O), RT_Rd); //  hat(omega)*R'*R_d
    temp2_v = mvmul(temp1_m, omega_d);      // (hat(omega)*R'*R_d)*omega_d
    temp3_v = mvmul(RT_Rd, domega_d);       // (R'*R_d*domega_d)

    temp4_v = mvmul(J, vsub(temp2_v, temp3_v)); // J*(hat(omega)*R'*R_d*omega_d - R'*R_d*domega_d)
    Gyro_dyn = vsub(temp1_v,temp4_v);

    F_thrust = vdot(F_thrust_ideal, b3);    // Project ideal thrust onto b3 vector [N]
    M = vadd(R_effort,Gyro_dyn);            // Control moments [Nm]
    //std::cout << "controlOutput is made" << std::endl;
}

float firstOrderFilter(float newValue, float prevValue, float alpha) {
    return alpha * newValue + (1 - alpha) * prevValue;
}