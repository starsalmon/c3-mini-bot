#pragma once

#include <Arduino.h>

// Bench-only ADC helper — fleet firmware has no battery code until this is calibrated.
class BatteryMonitor {
 public:
  void begin(int adc_pin, float scale, float low_v, float shutdown_v, uint32_t shutdown_hold_ms);
  // Returns true when it's time to deep-sleep (caller invokes bot_halt_low_battery).
  bool service(uint32_t now_ms);
  float voltage() const { return _volts; }
  uint16_t pin_millivolts() const { return _pin_mv; }
  // False when divider reads below BAT_VOLT_MIN_VALID (USB dev / pack unplugged).
  bool valid() const { return _valid; }
  bool low() const { return _low; }
  bool critical() const { return _critical; }

 private:
  void read_adc();

  int _pin = -1;
  float _scale = 3.7f;
  float _low_v = 3.5f;
  float _shutdown_v = 3.4f;
  uint32_t _shutdown_hold_ms = 2000;
  uint32_t _below_shutdown_since = 0;
  uint32_t _last_adc_ms = 0;
  uint32_t _boot_ms = 0;
  uint16_t _pin_mv = 0;
  bool _primed = false;
  float _volts = 0.0f;
  bool _valid = false;
  bool _low = false;
  bool _critical = false;
};
