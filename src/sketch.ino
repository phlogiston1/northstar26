// #include "Arduino_RouterBridge.h"
#include <cmath>

#define THRUST_COEFF 0.0001
#define QUADCOPTER_MASS 0.1
#define G 9.81



//GET USING compute_K.py
static const double LQR_K[4][12] = {
    {-0.000000,-0.000000,6.716873,-0.000000,-0.000000,6.571935,0.000000,-0.000000,-0.000000,0.000000,-0.000000,-0.000000},
    {-0.000000,-1.309546,0.000000,0.000000,-1.638736,0.000000,6.204574,0.000000,0.000000,1.127978,0.000000,0.000000},
    {1.309546,-0.000000,0.000000,1.638736,-0.000000,0.000000,0.000000,6.204574,-0.000000,0.000000,1.127978,-0.000000},
    {-0.000000,-0.000000,0.000000,-0.000000,-0.000000,-0.000000,0.000000,-0.000000,3.126511,0.000000,-0.000000,1.978831},
};

static const double LQR_KNEG[4][12] = {
    {0.000000,0.000000,-6.716873,0.000000,0.000000,-6.571935,-0.000000,0.000000,0.000000,-0.000000,0.000000,0.000000},
    {0.000000,1.309546,-0.000000,-0.000000,1.638736,-0.000000,-6.204574,-0.000000,-0.000000,-1.127978,-0.000000,-0.000000},
    {-1.309546,0.000000,-0.000000,-1.638736,0.000000,-0.000000,-0.000000,-6.204574,0.000000,-0.000000,-1.127978,0.000000},
    {0.000000,0.000000,-0.000000,0.000000,0.000000,0.000000,-0.000000,0.000000,-3.126511,-0.000000,0.000000,-1.978831},
};

//GET USING compute_mixer.py
static const double LQR_MIXER[4][4] = {
    {0.250000,3.535534,0.000000,-0.250000},
    {0.250000,0.000000,3.535534,0.250000},
    {0.250000,-3.535534,-0.000000,-0.250000},
    {0.250000,-0.000000,-3.535534,0.250000},
};

// 91 x 2 bytes ==> 182 bytes
unsigned int isinTable16[] = { 
  0, 1144, 2287, 3430, 4571, 5712, 6850, 7987, 9121, 10252, 11380, 
  12505, 13625, 14742, 15854, 16962, 18064, 19161, 20251, 21336, 22414, 
  23486, 24550, 25607, 26655, 27696, 28729, 29752, 30767, 31772, 32768, 

  33753, 34728, 35693, 36647, 37589, 38521, 39440, 40347, 41243, 42125, 
  42995, 43851, 44695, 45524, 46340, 47142, 47929, 48702, 49460, 50203, 
  50930, 51642, 52339, 53019, 53683, 54331, 54962, 55577, 56174, 56755, 

  57318, 57864, 58392, 58902, 59395, 59869, 60325, 60763, 61182, 61583, 
  61965, 62327, 62671, 62996, 63302, 63588, 63855, 64103, 64331, 64539, 
  64728, 64897, 65047, 65176, 65286, 65375, 65445, 65495, 65525, 65535, 
  };

double isin(long x){
    bool pos = true;  // positive - keeps an eye on the sign.
    if (x < 0) {
        x = -x;
        pos = false;
    }
    if (x >= 360) x %= 360;
    if (x > 180) {
        x -= 180;
        pos = !pos;
    }
    if (x > 90) x = 180 - x;
    if (pos) return isinTable16[x] * 0.0000152590219;
    return isinTable16[x] * -0.0000152590219;
}

double icos(long x){
    return isin(x+90);
}


double thrustToVel(double thrust) {
    return std::sqrt(std::abs(thrust) / THRUST_COEFF) * std::copysign(1.0, thrust);
}


//------------------------------------------------------------
// Utility vector functions (12-element)
//------------------------------------------------------------

void subtract12(const double a[12], const double b[12], double out[12]) {
    for (int i = 0; i < 12; i++)
        out[i] = a[i] - b[i];
}

void add4(const double a[4], const double b[4], double out[4]) {
    for (int i = 0; i < 4; i++)
        out[i] = a[i] + b[i];
}


//------------------------------------------------------------
// Matrix utilities
//------------------------------------------------------------

void negate4x12(const double in[4][12], double out[4][12]) {
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 12; ++j)
            out[i][j] = -in[i][j];
}

void mat4x12_vec12(const double mat[4][12], const double vec[12], double out[4]) {
    for (int i = 0; i < 4; i++) {
        out[i] = 0.0;
        for (int j = 0; j < 12; j++)
            out[i] += mat[i][j] * vec[j];
    }
}

void mat4x4_vec4(const double mat[4][4], const double vec[4], double out[4]) {
    for (int i = 0; i < 4; i++) {
        out[i] = 0.0;
        for (int j = 0; j < 4; j++)
            out[i] += mat[i][j] * vec[j];
    }
}


//------------------------------------------------------------
// LQR control step
//------------------------------------------------------------

double angle_limit = M_PI / 4;

void lqrControlStep(const double current[12], const double reference[12], double out[4]) {
    double error[12];
    subtract12(current, reference, error);

    // error = current - reference
    // current[8] = yaw

    double yaw = -current[8];
    // double cy = cos(yaw);
    // double sy = sin(yaw);
    //APPROXIMATIONS FOR SPEED
    double cy = icos(static_cast<long>(yaw*57.295779513));
    double sy = isin(static_cast<long>(yaw*57.295779513));

    //---------------------------------------------
    // Rotate the POSITION error into local frame
    //---------------------------------------------

    double posErr_x =  error[0];
    double posErr_y =  error[1];
    double posErr_z =  error[2];

    // Only yaw affects x/y:
    double posErr_local_x = cy * posErr_x - sy * posErr_y;
    double posErr_local_y = sy * posErr_x + cy * posErr_y;
    double posErr_local_z = posErr_z;


    //---------------------------------------------
    // Rotate the VELOCITY error into local frame
    //---------------------------------------------

    double velErr_x = error[3];
    double velErr_y = error[4];
    double velErr_z = error[5];

    double velErr_local_x = cy * velErr_x - sy * velErr_y;
    double velErr_local_y = sy * velErr_x + cy * velErr_y;
    double velErr_local_z = velErr_z;


    //---------------------------------------------
    // Build the 12-element local error vector
    //---------------------------------------------

    double error_local[12] = {
        posErr_local_x,
        posErr_local_y,
        posErr_local_z,
        velErr_local_x,
        velErr_local_y,
        velErr_local_z,
        error[6],   // roll error  (unchanged)
        error[7],   // pitch error (unchanged)
        error[8],   // yaw error   (unchanged!) per your original code
        error[9],   // roll rate
        error[10],  // pitch rate
        error[11]   // yaw rate    (unchanged)
    };

    // double error_local[12];
    // getStateVector(posErr, velErr, error_local);

    // keep yaw and yaw rate unchanged
    error_local[8] = error[8];
    error_local[11] = error[11];

    // double Kneg[4][12];
    // negate4x12(LQR_K, Kneg);

    double feedback[4];
    mat4x12_vec12(LQR_KNEG, error_local, feedback);

    double steady_state[4] = {
        QUADCOPTER_MASS * G,
        0, 0, 0
    };

    add4(feedback, steady_state, out);
}


//------------------------------------------------------------
// Mixer
//------------------------------------------------------------

void applyMixer(const double control[4], double out[4]) {
    double thrusts[4];
    mat4x4_vec4(LQR_MIXER, control, thrusts);

    out[0] = thrustToVel(thrusts[1]);  // left
    out[1] = thrustToVel(thrusts[0]);  // front
    out[2] = thrustToVel(thrusts[3]);  // right
    out[3] = thrustToVel(thrusts[2]);   // back
}

//{position x, position y, position z, velocity x, velocity y, velocity z, roll, pitch, yaw, roll rate, pitch rate, yaw rate}
double ref[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
double cur[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
double motor_velocities[4] = {0,0,0,0};


void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    //https://www.youtube.com/watch?v=vWpq636f1Z4
    Bridge.begin(460800);
    Bridge.provide("p", recieve_reference_pos);
    Bridge.provide("v", recieve_reference_vel);
    Bridge.provide("s", recieve_current_state);
}

bool state = false;

void loop() {
    // (Eventually) get IMU data, store in "cur" array (current state)

    double LQR_result[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
    lqrControlStep(cur, ref, LQR_result);
    applyMixer(LQR_result, motor_velocities);

    state = !state;
    set_led_state(state);
    // Transmit data to linux loop
    Bridge.call(
        "r",
        getLeft(),
        getFront(),
        getRight(),
        getBack(),
        0,
        0,
        0,
        0,
        0,
        0
    );
    // todo imu data
}

void set_led_state(bool state) {
    // LOW state means LED is ON
    digitalWrite(LED_BUILTIN, state ? LOW : HIGH);
}

void recieve_reference_pos(double pos_x,
                        double pos_y,
                        double pos_z,
                        double ang_pos_x,
                        double ang_pos_y,
                        double ang_pos_z) {
    ref[0]  = pos_x;
    ref[1]  = pos_y;
    ref[2]  = pos_z;
    ref[6]  = ang_pos_x;
    ref[7]  = ang_pos_y;
    ref[8]  = ang_pos_z;
}

void recieve_reference_vel(
                        double vel_x,
                        double vel_y,
                        double vel_z,
                        double ang_vel_x,
                        double ang_vel_y,
                        double ang_vel_z) {
    ref[3]  = vel_x;
    ref[4]  = vel_y;
    ref[5]  = vel_z;

    ref[9]  = ang_vel_x;
    ref[10] = ang_vel_y;
    ref[11] = ang_vel_z;
}


void recieve_current_state(
                        double cur_pos_x,
                        double cur_pos_y,
                        double cur_pos_z,
                        double cur_vel_x,
                        double cur_vel_y,
                        double cur_vel_z) {

    cur[0]  = cur_pos_x;
    cur[1]  = cur_pos_y;
    cur[2]  = cur_pos_z;

    cur[3]  = cur_vel_x;
    cur[4]  = cur_vel_y;
    cur[5]  = cur_vel_z;
}

double getLeft() {
    return motor_velocities[0];
}

double getFront() {
    return motor_velocities[1];
}

double getRight() {
    return motor_velocities[2];
}

double getBack() {
    return motor_velocities[3];
}