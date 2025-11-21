#include <vector>

/**
 * @brief All the constants needed to characterize the quadcopter physics and controls
 */

#define LOOP_TIME 0.018 //main loop time in seconds

#define G 9.81

// ----------- QUADCOPTER PHYSICAL CONSTANTS -------------
#define QUADCOPTER_ROTOR_DISTANCE 0.2 //quadcopter rotor center distance, in meters

#define QUADCOPTER_MASS 1.2 //mass in kg
#define QUADCOPTER_MOI 0.02 //moment of intertia of the quadcopter - OLD
#define QUADCOPTER_IXY 0.02 //moment of intertia of the quadcopter about X and Y axes, used for pitch and roll
#define QUADCOPTER_IZ 0.02 //moment of intertia of the quadcopter about Z axis, used for yaw


#define FRONT_LEFT_SPINS_CCW false //allows inverting assumed rotor spin directions to make it more flexible





// ----------- QUADCOPTER MOTION PROFILING CONSTANTS -------------
// these constants are arbitrary as long as they are within the physical limits of the quadcopter.
// They are used as the maximum velocities and accelerations for motion profiling the quadcopter's movement,
// As well as smoothing manual control inputs to ensure they are attainable.
// They will probably be relatively low to give the control system less aggressive commands, which should help with stability.
#define MAX_VELOCITY_XY 1.0 //max horizontal velocity in m/s
#define MAX_VELOCITY_Z 1.0 //max vertical velocity in m/s
#define MAX_ACCELERATION_XY 1.0 //max horizontal acceleration in m/s^2
#define MAX_ACCELERATION_Z 1.0 //max vertical acceleration in m/s^2
#define MAX_JERK_XY 0.3 //max horizontal jerk in m/s^3
#define MAX_YAW_RATE 1.0 //max yaw rate in rad/s
#define MAX_YAW_ACCELERATION 1.0 //max yaw acceleration in rad/s^2




// ----------- PHYSICS CONSTANTS -------------
// These constants are used in the physics calculations for the quadcopter model.
// Most of these should be empirically determined for the specific quadcopter design.
//Thrust calculation:
#define THRUST_COEFF 0.00001 //constant used to calculate rotor thrust. units: N per Rad/S.
//thrust calculation based on the formula: Thrust = THRUST_COEFF * AIR_DENSITY * ROTOR_AREA * (rotor velocity * ROTOR_RADIUS)^2
//which can be simplified to Thrust = THRUST_COEFF * rotor velocity^2 (where thrust coeff is empirically measured)

#define ROTOR_DRAG_COEFF 0.001 //This is used to calculate the angular force produced by the rotors about the Z axis.
//again the formula is DRAG_COEFF * rotor velocity^2

//super simple drag formula F = -kv^2. Simple, but might require manual tuning.
#define LINEAR_DRAG_COEFF_XY 0.5//0.25
#define LINEAR_DRAG_COEFF_Z 0.5//0.25
#define ANGULAR_DRAG_COEFF_XY 0.5
#define ANGULAR_DRAG_COEFF_Z 0.5





// ----------- MOTOR CONTROL CONSTANTS -------------
#define MOTOR_KS 0.0 //motor velocity static friction constant
#define MOTOR_KV 0.0 //motor velocity proportional constant
#define MOTOR_KA 0.0 //motor velocity acceleration constant
// #define MOTOR_KP 0.0 //motor velocity proportional control constant
// #define MOTOR_KI 0.0 //motor velocity integral control constant
// #define MOTOR_KD 0.0 //motor velocity derivative control constant
// #define MOTOR_INTEGRAL_WINDUP_LIMIT 100.0 //limit for motor velocity PID integral term to prevent windup
#define MOTOR_VELOCITY_RAMP_RATE 1000.0 //max change in motor velocity per second
#define MAX_MOTOR_VELOCITY 1000.0 //max motor velocity in Rad/s
#define ENABLE_INV_KIN_MOTOR_CONTSTRAINTS false

#define ENABLE_FLOOR true

#define VERBOSE false

