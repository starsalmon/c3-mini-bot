#pragma once

#include <Arduino.h>

class Ultrasonic {
 public:
  bool begin(int trig_pin, int echo_pin);
  // Returns range in metres, or -1 on timeout / invalid.
  float read_range_m(uint32_t timeout_us = 30000);
  uint32_t last_pulse_us() const { return _last_pulse_us; }
  void diagnose(Stream& out);

 private:
  uint32_t ping_once(uint32_t timeout_us);

  int _trig = -1;
  int _echo = -1;
  uint32_t _last_ping_ms = 0;
  uint32_t _last_pulse_us = 0;
};
