#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>

#include "bot_pins.h"

class DriveServos;

class DriveServos {
 public:
  bool begin(int left_pin, int right_pin);
  bool reattach(int left_pin, int right_pin);
  bool ok() const { return _ok; }
  void stop();
  void set_twist(float linear, float angular);
  void set_wheel_speeds(float left, float right);
  void tick();
  bool moving() const { return fabsf(_cur_l) > MOVE_EPS || fabsf(_cur_r) > MOVE_EPS; }

 private:
  void apply_current();
  void set_targets(float left, float right);
  uint16_t speed_to_us(float speed) const;

  Servo _left;
  Servo _right;
  bool _ok = false;
  uint16_t _last_us_l = 0;
  uint16_t _last_us_r = 0;
  float _trim_left = 1.0f;
  float _trim_right = 1.0f;
  float _tgt_l = 0.0f;
  float _tgt_r = 0.0f;
  float _cur_l = 0.0f;
  float _cur_r = 0.0f;

  static constexpr float MAX_STEP = DRIVE_MAX_STEP;
  static constexpr float SPIN_STEP = DRIVE_SPIN_STEP;
  static constexpr float MOVE_EPS = DRIVE_MOVE_EPS;
};
