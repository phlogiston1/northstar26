#include "WT901_IMU.h"

WT901_IMU::WT901_IMU(uint8_t address) : addr(address) {
    memset(&state, 0, sizeof(state));
}

bool WT901_IMU::begin() {
    Wire.begin();
    // give the sensor a moment
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
    return readVector(0x3D, ang, 180.0f);
}

bool WT901_IMU::readGyro(Vector3 &gyro) {
    return readVector(0x43, gyro, 2000.0f);
}

bool WT901_IMU::readAccel(Vector3 &accel) {
    return readVector(0x35, accel, 16.0f);
}

bool WT901_IMU::readMag(Vector3 &mag) {
    // magnetic units in some WT9xx variants are raw - keep scale 1.0
    return readVector(0x4B, mag, 1.0f);
}

bool WT901_IMU::readQuaternion(float q[4]) {
    uint8_t b[8];
    if (!readRawBytes(0x51, b, 8)) return false;

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
    if (!readRawBytes(0x41, b, 2)) return false;
    int16_t raw = (int16_t)(b[0] | (b[1] << 8));
    temp = raw / 100.0f;
    return true;
}

// ================= Non-blocking bulk read implementation =================
// We request a bulk block starting at register 0x35 (Accel X L) and read 33 bytes:
// layout (example): Acc(6) Gyro(6) Angle(6) Temp(2) Quaternion(8) Mag(6) = 34
// Different WT901 variants use slightly different layouts. This implementation reads 33 bytes
// and decodes in the order the common WT901 manual uses: Acc(6), Gyro(6), Angle(6), Temp(2), Quat(8), Mag(6)

bool WT901_IMU::update() {
    // Initiate a repeated-start bulk read from register 0x35
    Wire.beginTransmission(addr);
    Wire.write(0x35);
    if (Wire.endTransmission(false) != 0) return false;

    // Request BULK_LEN bytes. Note: on many Arduino cores this will block until bytes are received,
    // but using Wire.requestFrom with the noStop flag keeps the bus in a repeated-start condition.
    // The fetch() method checks Wire.available() and decodes only when all bytes are present.
    Wire.requestFrom((int)addr, (int)BULK_LEN, (int)false);
    return true;
}

bool WT901_IMU::fetch() {
    // Wait until full block present
    if (Wire.available() < BULK_LEN) return false;

    for (uint8_t i = 0; i < BULK_LEN; i++) nb_buf[i] = Wire.read();

    auto rd16 = [&](uint8_t idx)->int16_t {
        return (int16_t)(nb_buf[idx] | (nb_buf[idx+1] << 8));
    };

    // decode according to assumed layout
    // Accel
    state.accel.x = rd16(0) / 32768.0f * 16.0f;
    state.accel.y = rd16(2) / 32768.0f * 16.0f;
    state.accel.z = rd16(4) / 32768.0f * 16.0f;

    // Gyro
    state.gyro.x  = rd16(6)  / 32768.0f * 2000.0f;
    state.gyro.y  = rd16(8)  / 32768.0f * 2000.0f;
    state.gyro.z  = rd16(10) / 32768.0f * 2000.0f;

    // Angles
    state.angles.x = rd16(12) / 32768.0f * 180.0f;
    state.angles.y = rd16(14) / 32768.0f * 180.0f;
    state.angles.z = rd16(16) / 32768.0f * 180.0f;

    // Temp
    state.temperature = rd16(18) / 100.0f;

    // Quaternion (w,x,y,z)
    state.quaternion[0] = rd16(20) / 32768.0f;
    state.quaternion[1] = rd16(22) / 32768.0f;
    state.quaternion[2] = rd16(24) / 32768.0f;
    state.quaternion[3] = rd16(26) / 32768.0f;

    // Mag (if present at end)
    state.mag.x = rd16(28) / 1.0f;
    state.mag.y = rd16(30) / 1.0f;
    // note: if BULK_LEN==33 we might not have 32/33 indexes for z; guard:
    if (BULK_LEN >= 33) state.mag.z = rd16(32) / 1.0f; else state.mag.z = 0.0f;

    state.timestamp_ms = millis();
    state.is_new = true;

    return true;
}

IMUState WT901_IMU::getState() {
    IMUState copy = state;
    copy.is_new = false;
    state.is_new = false;
    return copy;
}