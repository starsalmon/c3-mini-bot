#include "drive_servos.h"

#include "bot_pins.h"
#include "servo_hw.h"

bool DriveServos::same_side_of_neutral(int16_t a, int16_t b, int16_t stop) {
  const int da = a - stop;
  const int db = b - stop;
  if (da == 0 || db == 0) {
    return true;
  }
  return (da > 0) == (db > 0);
}

int16_t DriveServos::slew_us_step(int16_t out, int16_t tgt, int16_t stop, uint16_t max_step) {
  int16_t eff_tgt = tgt;
  if (!same_side_of_neutral(out, tgt, stop) && out != stop) {
    eff_tgt = stop;
  }
  int32_t delta = static_cast<int32_t>(eff_tgt) - static_cast<int32_t>(out);
  if (delta > static_cast<int32_t>(max_step)) {
    delta = max_step;
  }
  if (delta < -static_cast<int32_t>(max_step)) {
    delta = -static_cast<int32_t>(max_step);
  }
  return static_cast<int16_t>(out + delta);
}

bool DriveServos::begin(int left_pin, int right_pin) {
  if (_ok) {
    return true;
  }
  _left_pin = left_pin;
  _right_pin = right_pin;
  _trim_left = SERVO_TRIM_LEFT;
  _trim_right = SERVO_TRIM_RIGHT;
  _stop_us_l = SERVO_STOP_US_LEFT;
  _stop_us_r = SERVO_STOP_US_RIGHT;
  _slew_us = SERVO_SLEW_US_PER_10MS;
  _max_offset_us = SERVO_MAX_OFFSET_US;
  servo_hw_init();
  servo_hw_prepare(_left);
  servo_hw_prepare(_right);
  _left.attach(left_pin, SERVO_MIN_US, SERVO_MAX_US);
  _right.attach(right_pin, SERVO_MIN_US, SERVO_MAX_US);
  _ok = _left.attached() && _right.attached();
  _detached = false;
  _out_us_l = _stop_us_l;
  _out_us_r = _stop_us_r;
  _last_us_l = _last_us_r = 0;
  _last_tick_ms = 0;
  _neutral_since_ms = 0;
  _hold_until_ms = 0;
  if (_ok) {
    stop();
  }
  return _ok;
}

bool DriveServos::reattach(int left_pin, int right_pin) {
  if (_ok) {
    _left.detach();
    _right.detach();
    _ok = false;
    _detached = false;
    _last_us_l = _last_us_r = 0;
  }
  return begin(left_pin, right_pin);
}

void DriveServos::set_motion_limits(uint16_t slew_us_per_10ms, uint16_t max_offset_us) {
  _slew_us = slew_us_per_10ms;
  _max_offset_us = constrain(max_offset_us, 50, 500);
}

uint16_t DriveServos::speed_to_us(float speed, bool left) const {
  if (speed > 1.0f) speed = 1.0f;
  if (speed < -1.0f) speed = -1.0f;
  const uint16_t stop_us = left ? _stop_us_l : _stop_us_r;
  if (fabsf(speed) <= MOVE_EPS) {
    return stop_us;
  }
  const float span = static_cast<float>(_max_offset_us);
  float us = static_cast<float>(stop_us) + speed * span;
  if (us < static_cast<float>(SERVO_MIN_US)) us = static_cast<float>(SERVO_MIN_US);
  if (us > static_cast<float>(SERVO_MAX_US)) us = static_cast<float>(SERVO_MAX_US);
  const int delta = static_cast<int>(us) - static_cast<int>(stop_us);
  if (abs(delta) <= SERVO_STOP_DEADBAND_US) {
    return stop_us;
  }
  return static_cast<uint16_t>(us);
}

void DriveServos::set_stop_us(uint16_t left_us, uint16_t right_us) {
  _stop_us_l = constrain(left_us, SERVO_MIN_US, SERVO_MAX_US);
  _stop_us_r = constrain(right_us, SERVO_MIN_US, SERVO_MAX_US);
  if (hold_stop_active() && !_detached) {
    _out_us_l = _stop_us_l;
    _out_us_r = _stop_us_r;
    write_outputs();
  }
}

bool DriveServos::at_neutral_pulse() const {
  const int dl = abs(static_cast<int>(_out_us_l) - static_cast<int>(_stop_us_l));
  const int dr = abs(static_cast<int>(_out_us_r) - static_cast<int>(_stop_us_r));
  return dl <= SERVO_STOP_DEADBAND_US && dr <= SERVO_STOP_DEADBAND_US;
}

bool DriveServos::moving() const {
  if (fabsf(_tgt_l) > MOVE_EPS || fabsf(_tgt_r) > MOVE_EPS) {
    return true;
  }
  return !at_neutral_pulse();
}

void DriveServos::write_outputs() {
  if (!_ok || _detached) {
    return;
  }
  if (_out_us_l != _last_us_l) {
    _left.writeMicroseconds(_out_us_l);
    _last_us_l = _out_us_l;
  }
  if (_out_us_r != _last_us_r) {
    _right.writeMicroseconds(_out_us_r);
    _last_us_r = _out_us_r;
  }
}

void DriveServos::apply_stop_hold() {
  _out_us_l = _stop_us_l;
  _out_us_r = _stop_us_r;
  write_outputs();
}

void DriveServos::ensure_attached() {
  if (!_ok || !_detached) {
    return;
  }
  servo_hw_prepare(_left);
  servo_hw_prepare(_right);
  _left.attach(_left_pin, SERVO_MIN_US, SERVO_MAX_US);
  _right.attach(_right_pin, SERVO_MIN_US, SERVO_MAX_US);
  _detached = false;
  _last_us_l = _last_us_r = 0;
  _neutral_since_ms = 0;
  write_outputs();
}

void DriveServos::detach_idle() {
  if (!_ok || _detached) {
    return;
  }
  _left.detach();
  _right.detach();
  if (_left_pin >= 0) {
    pinMode(_left_pin, OUTPUT);
    digitalWrite(_left_pin, LOW);
  }
  if (_right_pin >= 0) {
    pinMode(_right_pin, OUTPUT);
    digitalWrite(_right_pin, LOW);
  }
  _detached = true;
  _last_us_l = _last_us_r = 0;
  _neutral_since_ms = 0;
}

void DriveServos::set_targets(float left, float right) {
  left *= _trim_left;
  right *= _trim_right;
#if SERVO_INVERT_LEFT
  left = -left;
#endif
#if SERVO_INVERT_RIGHT
  right = -right;
#endif
  _tgt_l = left;
  _tgt_r = right;
  if (fabsf(_tgt_l) > MOVE_EPS || fabsf(_tgt_r) > MOVE_EPS) {
    ensure_attached();
  }
}

void DriveServos::stop() {
  _tgt_l = _tgt_r = 0.0f;
  if (!_ok) {
    return;
  }
}

void DriveServos::set_wheel_speeds(float left, float right) {
  set_targets(left, right);
}

void DriveServos::set_twist(float linear, float angular) {
  if (hold_stop_active()) {
    return;
  }
  float left = linear - angular;
  float right = linear + angular;
  const float peak = fmaxf(fabsf(left), fabsf(right));
  if (peak > 1.0f) {
    left /= peak;
    right /= peak;
  }
  set_targets(left, right);
}

void DriveServos::begin_hold_stop(uint32_t duration_ms) {
  _hold_until_ms = millis() + duration_ms;
  _tgt_l = _tgt_r = 0.0f;
  _neutral_since_ms = 0;
  ensure_attached();
}

void DriveServos::cancel_hold_stop() {
  _hold_until_ms = 0;
}

void DriveServos::tick_step(uint32_t now_ms) {
  if (!_ok) {
    return;
  }

  if (_hold_until_ms != 0 && now_ms >= _hold_until_ms) {
    _hold_until_ms = 0;
  }

  const bool hold = hold_stop_active();
  const bool want_drive = !hold && (fabsf(_tgt_l) > MOVE_EPS || fabsf(_tgt_r) > MOVE_EPS);

  uint16_t tgt_l = _stop_us_l;
  uint16_t tgt_r = _stop_us_r;
  if (want_drive) {
    ensure_attached();
    tgt_l = speed_to_us(_tgt_l, true);
    tgt_r = speed_to_us(_tgt_r, false);
  } else if (hold) {
    ensure_attached();
  }

  _out_us_l = static_cast<uint16_t>(
      slew_us_step(_out_us_l, tgt_l, _stop_us_l, _slew_us));
  _out_us_r = static_cast<uint16_t>(
      slew_us_step(_out_us_r, tgt_r, _stop_us_r, _slew_us));

  write_outputs();

  const bool stopped_cmd = !want_drive && !hold;
  if (stopped_cmd && at_neutral_pulse()) {
    if (_neutral_since_ms == 0) {
      _neutral_since_ms = now_ms;
    }
    if (!_detached && (now_ms - _neutral_since_ms) >= SERVO_IDLE_DETACH_MS) {
      detach_idle();
    }
  } else {
    _neutral_since_ms = 0;
  }
}

void DriveServos::tick(uint32_t now_ms) {
  if (!_ok) {
    return;
  }
  if (now_ms == 0) {
    now_ms = millis();
  }
  if (_last_tick_ms == 0) {
    _last_tick_ms = now_ms;
  }
  while (now_ms - _last_tick_ms >= SERVO_OUT_TICK_MS) {
    _last_tick_ms += SERVO_OUT_TICK_MS;
    tick_step(_last_tick_ms);
  }
}