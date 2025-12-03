#ifndef WT901_IMU_H
#define WT901_IMU_H


#include <Arduino.h>
#include <Wire.h>


// ================= Register Map =================
namespace WT901Reg {
    static const uint8_t ANGLE = 0x3D; // 6 bytes
    static const uint8_t GYRO = 0x43; // 6 bytes
    static const uint8_t ACCEL = 0x35; // 6 bytes
    static const uint8_t MAG = 0x4B; // 6 bytes
    static const uint8_t QUAT = 0x51; // 8 bytes
    static const uint8_t TEMP = 0x41; // 2 bytes
    static const uint8_t BULK_START = 0x35; // Bulk begins at Accel X L
};


struct Vector3 {
    float x, y, z;
};


struct IMUState {
    Vector3 angles;
    Vector3 gyro;
    Vector3 accel;
    Vector3 mag;
    float quaternion[4];
    float temperature;
    unsigned long timestamp_ms;
    bool is_new;
};


class WT901_IMU {
    public:
        // ===== Configuration API =====
        bool setBaudRate(uint16_t baud);
        bool setOutputRate(uint8_t hz);
        bool saveConfig();
        bool restoreFactory();

        WT901_IMU(uint8_t address = 0x50);
        bool begin();


        // Blocking reads
        bool readAngles(Vector3 &ang);
        bool readGyro(Vector3 &gyro);
        bool readAccel(Vector3 &accel);
        bool readMag(Vector3 &mag);
        bool readQuaternion(float q[4]);
        bool readTemperature(float &temp);


        // Non-blocking bulk read interface
        bool update();
        bool fetch();


        IMUState getState();


    private:
        // Low‑level config writer
        bool writeConfigCmd(uint8_t cmd, uint16_t value = 0);




        uint8_t addr;


        bool readVector(uint8_t reg, Vector3 &out, float scale);
        bool readRawBytes(uint8_t reg, uint8_t *buf, uint8_t len);


        static const uint8_t BULK_LEN = 33;
        uint8_t nb_buf[BULK_LEN];
        IMUState state;
};


#endif