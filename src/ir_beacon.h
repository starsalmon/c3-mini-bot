#pragma once

#include <Arduino.h>

// TSOP4838 etc.: output LOW when 38 kHz IR is present (use INPUT_PULLUP).
class IrBeacon {
 public:
  void begin(int left_pin, int right_pin);
  bool read_left() const;
  bool read_right() const;
  bool read_any() const;

 private:
  int _left = -1;
  int _right = -1;
};
