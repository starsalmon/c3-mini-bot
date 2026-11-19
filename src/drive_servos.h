#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>

#include "bot_pins.h"

class DriveServos {
 public:
  bool begin(int left_pin, int right_pin);
  bool reattach(int left_pin, int right_pin);
  bool ok() const { return _ok; }
  void stop();
  void set_twist(float linear, float angular);
  void set_wheel_speeds(float left, float right);
  void set_stop_us(uint16_t left_us, uint16_t right_us);
  void set_motion_limits(uint16_t slew_us_per_10ms, uint16_t max_offset_us);
  uint16_t stop_us_left() const { return _stop_us_l; }
  uint16_t stop_us_right() const { return _stop_us_r; }
  uint16_t slew_us_per_10ms() const { return _slew_us; }
  uint16_t max_offset_us() const { return _max_offset_us; }
  uint16_t out_us_left() const { return _out_us_l; }
  uint16_t out_us_right() const { return _out_us_r; }
  void begin_hold_stop(uint32_t duration_ms = SERVO_HOLD_STOP_MS);
  void cancel_hold_stop();
  bool hold_stop_active() const { return _hold_until_ms != 0 && millis() < _hold_until_ms; }
  void tick(uint32_t now_ms = 0);
  bool moving() const;
  bool detached() const { return _detached; }
  bool at_neutral_pulse() const;

 private:
  void tick_step(uint32_t now_ms);
  void apply_stop_hold();
  void write_outputs();
  void set_targets(float left, float right);
  void ensure_attached();
  void detach_idle();
  uint16_t speed_to_us(float speed, bool left) const;
  static int16_t slew_us_step(int16_t out, int16_t tgt, int16_t stop, uint16_t max_step);
  static bool same_side_of_neutral(int16_t a, int16_t b, int16_t stop);

  Servo _left;
  Servo _right;
  int _left_pin = -1;
  int _right_pin = -1;
  bool _ok = false;
  bool _detached = false;
  uint32_t _neutral_since_ms = 0;
  uint32_t _hold_until_ms = 0;
  uint32_t _last_tick_ms = 0;
  uint16_t _last_us_l = 0;
  uint16_t _last_us_r = 0;
  uint16_t _out_us_l = 0;
  uint16_t _out_us_r = 0;
  uint16_t _stop_us_l = SERVO_STOP_US_LEFT;
  uint16_t _stop_us_r = SERVO_STOP_US_RIGHT;
  uint16_t _slew_us = SERVO_SLEW_US_PER_10MS;
  uint16_t _max_offset_us = SERVO_MAX_OFFSET_US;
  float _trim_left = 1.0f;
  float _trim_right = 1.0f;
  float _tgt_l = 0.0f;
  float _tgt_r = 0.0f;

  static constexpr float MOVE_EPS = DRIVE_MOVE_EPS;
};