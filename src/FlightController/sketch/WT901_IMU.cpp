#include "WT901_IMU.h"

WT901_IMU::WT901_IMU(uint8_t address) : addr(address) {
    memset(&state, 0, sizeof(state));
}

bool WT901_IMU::begin() {
    Wire.begin();
    delay(20);
    state.is_new = false;
    return true;
}

bool WT901_IMU::readRawBytes(uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;

    uint8_t got = Wire.requestFrom((int)addr, (int)len);
    if (got != len) return false;

    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

bool WT901_IMU::readVector(uint8_t reg, Vector3 &out, float scl) {
    uint8_t b[6];
    if (!readRawBytes(reg, b, 6)) return false;

    int16_t x = (int16_t)(b[0] | (b[1] << 8));
    int16_t y = (int16_t)(b[2] | (b[3] << 8));
    int16_t z = (int16_t)(b[4] | (b[5] << 8));

    out.x = x / 32768.0f * scl;
    out.y = y / 32768.0f * scl;
    out.z = z / 32768.0f * scl;
    return true;
}

bool WT901_IMU::readAngles(Vector3 &ang) {
    return readVector(WT901Reg::ANGLE, ang, 180.0f);
}

bool WT901_IMU::readGyro(Vector3 &gyro) {
    return readVector(WT901Reg::GYRO, gyro, 2000.0f);
}

bool WT901_IMU::readAccel(Vector3 &accel) {
    return readVector(WT901Reg::ACCEL, accel, 16.0f);
}

bool WT901_IMU::readMag(Vector3 &mag) {
    return readVector(WT901Reg::MAG, mag, 1.0f);
}

bool WT901_IMU::readQuaternion(float q[4]) {
    uint8_t b[8];
    if (!readRawBytes(WT901Reg::QUAT, b, 8)) return false;

    int16_t w = (int16_t)(b[0] | (b[1] << 8));
    int16_t x = (int16_t)(b[2] | (b[3] << 8));
    int16_t y = (int16_t)(b[4] | (b[5] << 8));
    int16_t z = (int16_t)(b[6] | (b[7] << 8));

    q[0] = w / 32768.0f;
    q[1] = x / 32768.0f;
    q[2] = y / 32768.0f;
    q[3] = z / 32768.0f;
    return true;
}

bool WT901_IMU::readTemperature(float &temp) {
    uint8_t b[2];
    if (!readRawBytes(WT901Reg::TEMP, b, 2)) return false;
    int16_t raw = (int16_t)(b[0] | (b[1] << 8));
    temp = raw / 100.0f;
    return true;
}

// ================= Non-blocking bulk read =================

bool WT901_IMU::update() {
    Wire.beginTransmission(addr);
    Wire.write(WT901Reg::BULK_START);
    if (Wire.endTransmission(false) != 0) return false;

    Wire.requestFrom((int)addr, (int)BULK_LEN, (int)false);
    return true;
}

bool WT901_IMU::fetch() {
    if (Wire.available() < BULK_LEN) return false;

    for (uint8_t i = 0; i < BULK_LEN; i++) nb_buf[i] = Wire.read();

    auto rd16 = [&](uint8_t idx)->int16_t {
        return (int16_t)(nb_buf[idx] | (nb_buf[idx+1] << 8));
    };

    state.accel.x = rd16(0) / 32768.0f * 16.0f;
    state.accel.y = rd16(2) / 32768.0f * 16.0f;
    state.accel.z = rd16(4) / 32768.0f * 16.0f;

    state.gyro.x  = rd16(6)  / 32768.0f * 2000.0f;
    state.gyro.y  = rd16(8)  / 32768.0f * 2000.0f;
    state.gyro.z  = rd16(10) / 32768.0f * 2000.0f;

    state.angles.x = rd16(12) / 32768.0f * 180.0f;
    state.angles.y = rd16(14) / 32768.0f * 180.0f;
    state.angles.z = rd16(16) / 32768.0f * 180.0f;

    state.temperature = rd16(18) / 100.0f;

    state.quaternion[0] = rd16(20) / 32768.0f;
    state.quaternion[1] = rd16(22) / 32768.0f;
    state.quaternion[2] = rd16(24) / 32768.0f;
    state.quaternion[3] = rd16(26) / 32768.0f;

    state.mag.x = rd16(28);
    state.mag.y = rd16(30);
    if (BULK_LEN >= 33) state.mag.z = rd16(32); else state.mag.z = 0.0f;

    state.timestamp_ms = millis();
    state.is_new = true;
    return true;
}

IMUState WT901_IMU::getState() {
    IMUState copy = state;
    state.is_new = false;
    return copy;
}