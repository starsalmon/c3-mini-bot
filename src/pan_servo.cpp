#include "pan_servo.h"

#include "bot_pins.h"
#include "servo_hw.h"

bool PanServo::begin(int pin) {
  if (_ok) {
    return true;
  }
  servo_hw_init();
  _servo.attach(pin, 1000, 2000);
  _ok = _servo.attached();
  if (_ok) {
    set_deg(PAN_CENTER_DEG);
  }
  return _ok;
}

bool PanServo::reattach(int pin) {
  const float deg = _deg;
  if (_ok) {
    _servo.detach();
    _ok = false;
    _last_us = 0;
  }
  if (!begin(pin)) {
    return false;
  }
  set_deg(deg);
  return true;
}

uint16_t PanServo::deg_to_us(float deg) const {
  if (deg < PAN_MIN_DEG) deg = PAN_MIN_DEG;
  if (deg > PAN_MAX_DEG) deg = PAN_MAX_DEG;
  const float norm = (deg - PAN_MIN_DEG) / (PAN_MAX_DEG - PAN_MIN_DEG);
  return static_cast<uint16_t>(1000.0f + norm * 1000.0f);
}

void PanServo::set_deg(float deg) {
  _deg = deg;
  writeMicroseconds(deg_to_us(deg));
}

void PanServo::writeMicroseconds(uint16_t us) {
  if (!_ok || us == _last_us) return;
  _servo.writeMicroseconds(us);
  _last_us = us;
}
