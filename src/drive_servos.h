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
  void set_stop_us(uint16_t left_us, uint16_t right_us);
  uint16_t stop_us_left() const { return _stop_us_l; }
  uint16_t stop_us_right() const { return _stop_us_r; }
  void tick();
  bool moving() const { return fabsf(_cur_l) > MOVE_EPS || fabsf(_cur_r) > MOVE_EPS; }
  bool detached() const { return _detached; }

 private:
  void apply_current();
  void apply_stop_hold();
  void set_targets(float left, float right);
  void ensure_attached();
  void detach_idle();
  uint16_t speed_to_us(float speed, bool left) const;

  Servo _left;
  Servo _right;
  int _left_pin = -1;
  int _right_pin = -1;
  bool _ok = false;
  bool _detached = false;
  uint32_t _idle_since_ms = 0;
  uint16_t _last_us_l = 0;
  uint16_t _last_us_r = 0;
  uint16_t _stop_us_l = SERVO_STOP_US_LEFT;
  uint16_t _stop_us_r = SERVO_STOP_US_RIGHT;
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
