#include "IMU.h"

// Official WT901 register scaling (from datasheet / protocol)
static constexpr float ACC_SCALE   = 16.0f   / 32768.0f;  // g
static constexpr float GYRO_SCALE  = 2000.0f / 32768.0f;  // deg/s
static constexpr float ANG_SCALE   = 180.0f  / 32768.0f;  // deg
static constexpr float TEMP_SCALE  = 1.0f    / 100.0f;    // deg C

// WT901 register map (each entry is 2 bytes, little-endian)
static constexpr uint8_t REG_ACC_X   = 0x34;
static constexpr uint8_t REG_GYRO_X  = 0x37;
static constexpr uint8_t REG_MAG_X   = 0x3A;
static constexpr uint8_t REG_ROLL    = 0x3D;
static constexpr uint8_t REG_TEMP    = 0x40;

WT901::WT901(uint8_t i2c_addr, TwoWire &wire) {
    _addr = i2c_addr;
    _wire = &wire;
}

void WT901::begin(uint32_t i2c_clock) {
    _wire->begin();
    _wire->setClock(i2c_clock);
}

bool WT901::isConnected() {
    _wire->beginTransmission(_addr);
    return (_wire->endTransmission() == 0);
}

// Non-blocking request flag (actual I2C happens in update)
void WT901::requestData() {
    if (_state != State::IDLE) return;
    _state = State::READING;
}

bool WT901::update() {
    if (_state != State::READING) return false;

    // Read 26 bytes starting at 0x34:
    // ACC(6) + GYRO(6) + MAG(6) + ANGLE(6) + TEMP(2)
    const uint8_t START_REG = REG_ACC_X;
    const uint8_t COUNT = 26;
    uint8_t buf[COUNT];

    // Set register pointer
    _wire->beginTransmission(_addr);
    _wire->write(START_REG);
    if (_wire->endTransmission(false) != 0) {
        _state = State::IDLE;
        return false;
    }

    // Read block
    if (_wire->requestFrom((int)_addr, (int)COUNT) != COUNT) {
        _state = State::IDLE;
        return false;
    }

    for (uint8_t i = 0; i < COUNT; ++i)
        buf[i] = _wire->read();

    auto s16 = [](uint8_t lo, uint8_t hi) -> int16_t {
        return (int16_t)((hi << 8) | lo);
    };

    // Offsets relative to 0x34
    const uint8_t ACC_O  = 0;   // 0x34
    const uint8_t GYR_O  = 6;   // 0x37
    const uint8_t MAG_O  = 12;  // 0x3A
    const uint8_t ANG_O  = 18;  // 0x3D
    const uint8_t TMP_O  = 24;  // 0x40

    // Acceleration
    _data.acc_x = s16(buf[ACC_O + 0], buf[ACC_O + 1]) * ACC_SCALE;
    _data.acc_y = s16(buf[ACC_O + 2], buf[ACC_O + 3]) * ACC_SCALE;
    _data.acc_z = s16(buf[ACC_O + 4], buf[ACC_O + 5]) * ACC_SCALE;

    // Gyroscope
    _data.gyro_x = s16(buf[GYR_O + 0], buf[GYR_O + 1]) * GYRO_SCALE;
    _data.gyro_y = s16(buf[GYR_O + 2], buf[GYR_O + 3]) * GYRO_SCALE;
    _data.gyro_z = s16(buf[GYR_O + 4], buf[GYR_O + 5]) * GYRO_SCALE;

    // Magnetometer (raw units)
    _data.mag_x = s16(buf[MAG_O + 0], buf[MAG_O + 1]);
    _data.mag_y = s16(buf[MAG_O + 2], buf[MAG_O + 3]);
    _data.mag_z = s16(buf[MAG_O + 4], buf[MAG_O + 5]);

    // Angles (degrees, ±180)
    _data.roll  = s16(buf[ANG_O + 0], buf[ANG_O + 1]) * ANG_SCALE;
    _data.pitch = s16(buf[ANG_O + 2], buf[ANG_O + 3]) * ANG_SCALE;
    _data.yaw   = s16(buf[ANG_O + 4], buf[ANG_O + 5]) * ANG_SCALE;

    // Temperature
    _data.temperature = s16(buf[TMP_O + 0], buf[TMP_O + 1]) * TEMP_SCALE;

    _data.timestamp_ms = millis();
    _state = State::IDLE;
    return true;
}
