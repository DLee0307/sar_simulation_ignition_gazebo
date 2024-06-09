#ifndef SHARED_LIB_H
#define SHARED_LIB_H

#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "sar_msgs/srv/ctrl_cmd_srv.hpp"

#include "math3d.h"
#include <cmath>

#include "Controller_GTC.h"
#include "Traj_Funcs.h"
#include "stabilizer.h"

#define PWM_MAX 60000
#define g2Newton (9.81f/1000.0f)
#define Newton2g (1000.0f/9.81f)
#define Deg2Rad (float)M_PI/180.0f
#define Rad2Deg 180.0f/(float)M_PI


// =================================
//    INITIAL SYSTEM PARAMETERS
// =================================
extern float m;                 // [kg]
extern float Ixx;               // [kg*m^2]
extern float Iyy;               // [kg*m^2]
extern float Izz;               // [kg*m^2]
extern struct mat33 J;          // Rotational Inertia Matrix [kg*m^2]

extern float C_tf;              // Moment Coeff [Nm/N]
extern float Thrust_max;        // Max thrust per motor [g]

extern float dt;                // Controller cycle time
extern uint32_t prev_tick;

// =================================
//    ROS2 PARAMETER
// =================================
extern float Gamma_eff; 
extern float K_Pitch; 
extern float K_Yaw;

extern std::string Plane_Type;
extern std::string Plane_Config;
struct Position {
    float x;
    float y;
    float z;
};

extern std::string POLICY_TYPE;

extern int Vicon_Delay_ms;

extern float SIM_SPEED;
extern float SIM_SLOWDOWN_SPEED;
extern bool LANDING_SLOWDOWN_FLAG;

extern int LOGGING_RATE;
extern bool SHOW_CONSOLE;

extern float Leg_Length;
extern int Leg_Angle;

extern float Tau_up;
extern float Tau_down;
extern std::vector<double> TrajAcc_Max;
extern std::vector<double> TrajJerk_Max;

extern float Base_Mass;             // [kg]
extern float Base_Ixx;           // [kg*m^2]
extern float Base_Iyy;           // [kg*m^2]
extern float Base_Izz;           // [kg*m^2]

// =================================
//       GEOMETRIC PARAMETERS
// =================================

extern float Prop_14_x;         // Front Prop Distance - x-axis [m]
extern float Prop_14_y;         // Front Prop Distance - y-axis [m]
extern float Prop_23_x;         // Rear  Prop Distance - x-axis [m]
extern float Prop_23_y;         // Rear  Prop Distance - y-axis [m]

extern float L_eff;             // Effective Leg Length [m]
extern float Forward_Reach;     // Forward Reach [m]
extern float Collision_Radius;  // Collision Radius [m]


// =================================
//    CONTROL GAIN DECLARATIONS
// =================================
// XY POSITION PID
extern float P_kp_xy;
extern float P_kd_xy;
extern float P_ki_xy;
extern float i_range_xy;

// Z POSITION PID
extern float P_kp_z;
extern float P_kd_z;
extern float P_ki_z;
extern float i_range_z;

// XY ATTITUDE PID
extern float R_kp_xy;
extern float R_kd_xy;
extern float R_ki_xy;
extern float i_range_R_xy;

// Z ATTITUDE PID
extern float R_kp_z;
extern float R_kd_z;
extern float R_ki_z;
extern float i_range_R_z;


// DECLARE CTRL GAIN VECTORS 
extern struct vec Kp_p;     // Pos. Proportional Gains 
extern struct vec Kd_p;     // Pos. Derivative Gains
extern struct vec Ki_p;     // Pos. Integral Gains  

extern struct vec Kp_R;     // Rot. Proportional Gains
extern struct vec Kd_R;     // Rot. Derivative Gains
extern struct vec Ki_R;     // Rot. Integral Gains


// DECLARE CTRL GAIN FLAGS
extern float kp_xf;         // Pos. Gain Flag
extern float kd_xf;         // Pos. Derivative Gain Flag


// =================================
//     BODY WRT ORIGIN STATES
// =================================
extern struct vec Pos_B_O;          // Pos [m]
extern struct vec Vel_B_O;          // Vel [m/s]
extern struct vec Accel_B_O;        // Linear Accel. [m/s^2]
extern float Accel_B_O_Mag;         // Linear Accel. Magnitude [m/s^2]

extern struct quat Quat_B_O;        // Orientation
extern struct vec Omega_B_O;        // Angular Rate [rad/s]
extern struct vec Omega_B_O_prev;   // Prev Angular Rate [rad/s^2]
extern struct vec dOmega_B_O;       // Angular Accel [rad/s^2]

extern struct mat33 R;              // Orientation as rotation matrix
extern struct vec b3;               // Current body z-axis in global coord.


// =================================
//     BODY WRT PLANE STATES
// =================================
extern struct vec Pos_P_B;         // Pos [m]
extern struct vec Vel_B_P;         // Vel [m/s]
extern struct quat Quat_P_B;       // Orientation
extern struct vec Omega_B_P;       // Angular Rate [rad/s]

// RELATIVE STATES
extern float D_perp;               // Distance from body to plane [m]
extern float D_perp_CR;            // Distance from CR to plane [m]
extern float Vel_mag_B_P;          // Velocity magnitude relative [m/s]
extern float Vel_angle_B_P;        // Velocity angle relative [deg]


// =================================
//         DESIRED STATES
// =================================
extern struct vec x_d;          // Pos-desired [m]
extern struct vec v_d;          // Vel-desired [m/s]
extern struct vec a_d;          // Acc-desired [m/s^2]

extern struct vec b1_d;         // Desired body x-axis in global coord. 
extern struct vec b2_d;         // Desired body y-axis in global coord.
extern struct vec b3_d;         // Desired body z-axis in global coord.
extern struct mat33 R_d;        // Desired rotational matrix from b_d vectors

extern struct quat quat_d;      // Orientation-desired [qx,qy,qz,qw]
extern struct vec omega_d;      // Omega-desired [rad/s]
extern struct vec domega_d;     // Ang. Acc-desired [rad/s^2]

// =================================
//         STATE ERRORS
// =================================
extern struct vec e_x;  // Pos-error [m]
extern struct vec e_v;  // Vel-error [m/s]
extern struct vec e_PI; // Pos. Integral-error [m*s]

extern struct vec e_R;  // Rotation-error [rad]
extern struct vec e_w;  // Omega-error [rad/s]
extern struct vec e_RI; // Rot. Integral-error [rad*s]


// =================================
//       CONTROLLER ACTUATIONS
// =================================
extern struct vec F_thrust_ideal;   // Ideal thrust vector [N]
extern float F_thrust;              // Implemented body thrust [N]
extern struct vec M;                // Implemented body moments [N*m]
extern struct vec M_d;              // Desired body moment [N*m]

// MOTOR THRUST ACTIONS
extern float f_thrust_g;        // Motor thrust - Thrust [g]
extern float f_roll_g;          // Motor thrust - Roll   [g]
extern float f_pitch_g;         // Motor thrust - Pitch  [g]
extern float f_yaw_g;           // Motor thrust - Yaw    [g]

// MOTOR THRUSTS
extern float M1_thrust;
extern float M2_thrust;
extern float M3_thrust;
extern float M4_thrust;

// MOTOR PWM VALUES
extern uint16_t M1_pwm; 
extern uint16_t M2_pwm; 
extern uint16_t M3_pwm; 
extern uint16_t M4_pwm; 

// CONTROL OVERRIDE VALUES
extern uint16_t PWM_override[4];    // Motor PWM values
extern float thrust_override[4];    // Motor thrusts [g] 

// =================================
//  FLAGS AND SYSTEM INITIALIZATION
// =================================

// CONTROLLER FLAGS
extern bool Tumbled_Flag;
extern bool TumbleDetect_Flag;
extern bool MotorStop_Flag;
extern bool AngAccel_Flag;
extern bool Armed_Flag;
extern bool CustomThrust_Flag;
extern bool CustomMotorCMD_Flag;
extern uint16_t CMD_ID;

// SENSOR FLAGS
extern bool CamActive_Flag;

// =================================
//       POLICY INITIALIZATION
// =================================

// POLICY SETTING
typedef enum {
    PARAM_OPTIM = 0,
    DEEP_RL_SB3 = 1,
    DEEP_RL_ONBOARD = 2,
}PolicyType;
extern PolicyType Policy;


// POLICY FLAGS
extern bool Policy_Armed_Flag;
extern bool Trg_Flag;
extern bool onceFlag;

// =================================
//    LANDING SURFACE PARAMETERS
// =================================
extern float Plane_Angle_deg;   // Plane Angle [deg]
extern struct vec r_P_O;        // Plane Position Vector        [m]

// CTRL COMMAND PACKETS
struct CTRL_CmdPacket{
    uint8_t cmd_type; 
    float cmd_val1;
    float cmd_val2;
    float cmd_val3;
    float cmd_flag;
    bool  cmd_rx;
} __attribute__((packed));
extern struct CTRL_CmdPacket CTRL_Cmd;



void CTRL_Command(struct CTRL_CmdPacket *CTRL_Cmd);
void controlOutput(const state_t *state, const sensorData_t *sensors);
float firstOrderFilter(float newValue, float prevValue, float alpha);



// =================================
//    ADDITIONAL MATH3D FUNCTIONS
// =================================

// Construct a matrix A from vector v such that Ax = cross(v, x)
static inline struct mat33 hat(struct vec v) {
	struct mat33 m;
	m.m[0][0] = 0;
	m.m[0][1] = -v.z;
	m.m[0][2] = v.y;
	m.m[1][0] = v.z;
	m.m[1][1] = 0;
	m.m[1][2] = -v.x;
	m.m[2][0] = -v.y;
	m.m[2][1] = v.x;
	m.m[2][2] = 0;
	return m;
}

// Construct a vector v from matrix A such that Ax = cross(v, x)
static inline struct vec dehat(struct mat33 m) {
	struct vec v;

	v.x = m.m[2][1];
	v.y = m.m[0][2];
	v.z = m.m[1][0];
	
	return v;
}

// Convert quaternion to (roll, pitch, yaw) Euler angles using Tait-Bryan convention [YZX]
//  - Pitch, then yaw about new pitch axis, then roll about new roll axis
//  - Notation allows greater than 90 deg pitch and roll angles
static inline struct vec quat2eul(struct quat q) {
	// from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

	struct vec eul;
	float R11,R21,R31,R22,R23;


	// CALC NEEDED ROTATION MATRIX COMPONENTS FROM QUATERNION
    R11 = 1.0f - 2.0f*( fsqr(q.y) + fsqr(q.z) );
    R21 = 2.0f*(q.x*q.y + q.z*q.w);
    R31 = 2.0f*(q.x*q.z - q.y*q.w);

    R22 = 1.0f - 2.0f*( fsqr(q.x) + fsqr(q.z) );
    R23 = 2.0f*(q.y*q.z - q.x*q.w);


	// CONVERT ROTATION MATRIX COMPONENTS TO EULER ANGLES (YZX NOTATION)
	eul.x = atan2f(-R23,R22); 	// Roll
	eul.y = atan2f(-R31,R11); 	// Pitch
	eul.z = asinf(R21); 		// Yaw

	return eul;
}

static inline void printvec(struct vec v){
	std::cout << v.x << ", " << v.y << ", " << v.z << std::endl;
	return;
}

static inline void printquat(struct quat q){
	std::cout << q.x << ", " << q.y << ", " << q.z << ", " << q.w << std::endl;
	return;
}

static inline void printmat(struct mat33 m){
    struct vec vrow_0 = mrow(m,0);
    struct vec vrow_1 = mrow(m,1);
    struct vec vrow_2 = mrow(m,2);

    printvec(vrow_0);
    printvec(vrow_1);
    printvec(vrow_2);
	std::cout << std::endl;

	return;
}


#endif // SHARED_LIB_H