#pragma once

#include <Arduino.h>

struct ImuSample {
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;
};

class Mpu6050Imu {
 public:
  bool begin(int sda_pin, int scl_pin, uint8_t addr = 0x68);
  bool ok() const { return _ok; }
  uint8_t addr() const { return _addr; }
  bool read(ImuSample& out);

 private:
  bool _ok = false;
  uint8_t _addr = 0x68;
  bool wake(uint8_t addr);
  bool write_byte(uint8_t reg, uint8_t value) const;
  bool read_bytes(uint8_t reg, uint8_t* buf, size_t len) const;
};
