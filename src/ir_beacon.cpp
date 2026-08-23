#include "ir_beacon.h"

void IrBeacon::begin(int left_pin, int right_pin) {
  _left = left_pin;
  _right = right_pin;
  if (_left >= 0) {
    pinMode(_left, INPUT_PULLUP);
  }
  if (_right >= 0) {
    pinMode(_right, INPUT_PULLUP);
  }
}

bool IrBeacon::read_left() const {
  return _left >= 0 && digitalRead(_left) == LOW;
}

bool IrBeacon::read_right() const {
  return _right >= 0 && digitalRead(_right) == LOW;
}

bool IrBeacon::read_any() const {
  return read_left() || read_right();
}
