#ifndef WT901_IMU_H
#define WT901_IMU_H

#include <Arduino.h>
#include <Wire.h>

struct Vector3 {
    float x, y, z;
};

struct IMUState {
    Vector3 angles;    // deg
    Vector3 gyro;      // deg/s
    Vector3 accel;     // g
    Vector3 mag;       // uT (raw scale)
    float quaternion[4]; // w,x,y,z (unitless)
    float temperature; // C
    unsigned long timestamp_ms; // millis()
    bool is_new;       // true if state updated since last get
};

class WT901_IMU {
public:
    WT901_IMU(uint8_t address = 0x50);
    bool begin();

    // Blocking reads
    bool readAngles(Vector3 &ang);       // Roll, Pitch, Yaw (deg)
    bool readGyro(Vector3 &gyro);        // deg/sec
    bool readAccel(Vector3 &accel);      // g
    bool readMag(Vector3 &mag);          // uT (raw)
    bool readQuaternion(float q[4]);     // w,x,y,z (unitless)
    bool readTemperature(float &temp);   // Celsius

    // Non-blocking API
    // Call update() periodically (e.g. every sample period). It will issue the I2C read request.
    // Then call fetch() repeatedly (or in the next loop) until it returns true, which means the
    // requested bytes have been received and decoded into the internal state.
    bool update(); // request a bulk read (does not copy into state)
    bool fetch();  // returns true when new state available (decodes into internal state)

    // Access latest decoded state (copy). After calling getState(), is_new will be cleared.
    IMUState getState();

private:
    uint8_t addr;
    float scale;

    // low-level helpers
    bool readVector(uint8_t reg, Vector3 &out, float scale);
    bool readRawBytes(uint8_t reg, uint8_t *buf, uint8_t len);

    // internal non-blocking buffer & state
    static const uint8_t BULK_LEN = 33; // full accel+gyro+angle+temp+quat (example)
    uint8_t nb_buf[BULK_LEN];
    IMUState state;
};

#endif