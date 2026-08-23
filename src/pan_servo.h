#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>

class PanServo {
 public:
  bool begin(int pin);
  bool reattach(int pin);
  bool ok() const { return _ok; }
  void set_deg(float deg);
  void writeMicroseconds(uint16_t us);
  float deg() const { return _deg; }

 private:
  Servo _servo;
  bool _ok = false;
  float _deg = 90.0f;
  uint16_t _last_us = 0;
  uint16_t deg_to_us(float deg) const;
};
