#ifndef WT901_H
#define WT901_H

#include <Arduino.h>
#include <Wire.h>

class WT901 {
public:
    struct Data {
        float acc_x, acc_y, acc_z;     // g
        float gyro_x, gyro_y, gyro_z;  // deg/s
        float mag_x, mag_y, mag_z;     // raw magnetometer units
        float roll, pitch, yaw;        // degrees (±180)
        float temperature;             // deg C
        unsigned long timestamp_ms;    // millis()
    };

    explicit WT901(uint8_t i2c_addr = 0x50, TwoWire &wire = Wire);

    void begin(uint32_t i2c_clock = 400000);
    bool isConnected();

    // Non-blocking API
    void requestData();   // arm a register read
    bool update();        // perform I2C read + parse if armed

    inline const Data &getData() const { return _data; }

  private:
      uint8_t _addr;
      TwoWire *_wire;
      Data _data{};
  
      enum class State {
          IDLE,
          READING
      } _state = State::IDLE;
};

#endif
