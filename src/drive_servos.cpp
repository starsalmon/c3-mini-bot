#include "drive_servos.h"

#include "bot_pins.h"
#include "servo_hw.h"

bool DriveServos::begin(int left_pin, int right_pin) {
  if (_ok) {
    return true;
  }
  _trim_left = SERVO_TRIM_LEFT;
  _trim_right = SERVO_TRIM_RIGHT;
  servo_hw_init();
  _left.attach(left_pin, SERVO_MIN_US, SERVO_MAX_US);
  _right.attach(right_pin, SERVO_MIN_US, SERVO_MAX_US);
  _ok = _left.attached() && _right.attached();
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
    _last_us_l = _last_us_r = 0;
  }
  return begin(left_pin, right_pin);
}

uint16_t DriveServos::speed_to_us(float speed) const {
  if (speed > 1.0f) speed = 1.0f;
  if (speed < -1.0f) speed = -1.0f;
  const float span = static_cast<float>(SERVO_MAX_US - SERVO_MIN_US) * 0.5f;
  const float us = static_cast<float>(SERVO_STOP_US) + speed * span;
  if (us < static_cast<float>(SERVO_MIN_US)) return SERVO_MIN_US;
  if (us > static_cast<float>(SERVO_MAX_US)) return SERVO_MAX_US;
  return static_cast<uint16_t>(us);
}

void DriveServos::apply_current() {
  if (!_ok) return;
  const uint16_t us_l = speed_to_us(_cur_l);
  const uint16_t us_r = speed_to_us(_cur_r);
  if (us_l != _last_us_l) {
    _left.writeMicroseconds(us_l);
    _last_us_l = us_l;
  }
  if (us_r != _last_us_r) {
    _right.writeMicroseconds(us_r);
    _last_us_r = us_r;
  }
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
}

void DriveServos::stop() {
  _tgt_l = _tgt_r = _cur_l = _cur_r = 0.0f;
  if (!_ok) return;
  const uint16_t us = SERVO_STOP_US;
  if (_last_us_l != us) {
    _left.writeMicroseconds(us);
    _last_us_l = us;
  }
  if (_last_us_r != us) {
    _right.writeMicroseconds(us);
    _last_us_r = us;
  }
}

void DriveServos::set_wheel_speeds(float left, float right) {
  set_targets(left, right);
}

void DriveServos::set_twist(float linear, float angular) {
  float left = linear - angular;
  float right = linear + angular;
  const float peak = fmaxf(fabsf(left), fabsf(right));
  if (peak > 1.0f) {
    left /= peak;
    right /= peak;
  }
  set_targets(left, right);
}

void DriveServos::tick() {
  if (!_ok) return;

  const bool spinning =
      (_tgt_l * _tgt_r < 0.0f) &&
      (fabsf(_tgt_l) > MOVE_EPS || fabsf(_tgt_r) > MOVE_EPS);
  const float max_step = spinning ? SPIN_STEP : MAX_STEP;

  auto step_side = [&](float& cur, float tgt) {
    float eff_tgt = tgt;
    if (cur * eff_tgt < -MOVE_EPS * MOVE_EPS) {
      eff_tgt = 0.0f;
    }
    const float step_lim =
        (cur * eff_tgt < 0.0f && fabsf(cur) > MOVE_EPS) ? SPIN_STEP : max_step;
    float delta = eff_tgt - cur;
    if (delta > step_lim) delta = step_lim;
    if (delta < -step_lim) delta = -step_lim;
    cur += delta;
  };

  step_side(_cur_l, _tgt_l);
  step_side(_cur_r, _tgt_r);

  if (spinning) {
    const float mag = fmaxf(fabsf(_cur_l), fabsf(_cur_r));
    if (mag > MOVE_EPS) {
      const float sign = (_tgt_r > 0.0f) ? 1.0f : -1.0f;
      _cur_l = -sign * mag;
      _cur_r = sign * mag;
    }
  }

  apply_current();
}
