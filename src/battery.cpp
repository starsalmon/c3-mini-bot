#include "battery.h"

#include <math.h>

#include "bot_pins.h"

void BatteryMonitor::begin(int adc_pin, float scale, float low_v, float shutdown_v,
                           uint32_t shutdown_hold_ms) {
  _pin = adc_pin;
  _scale = scale;
  _low_v = low_v;
  _shutdown_v = shutdown_v;
  _shutdown_hold_ms = shutdown_hold_ms;
  _below_shutdown_since = 0;
  _last_adc_ms = 0;
  _boot_ms = 0;
  _primed = false;
  _pin_mv = 0;
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  for (int i = 0; i < 12; i++) {
    (void)analogReadMilliVolts(_pin);
    delay(5);
  }
  _volts = 0.0f;
  _valid = false;
  _low = false;
  _critical = false;
}

namespace {

uint32_t median_mv(uint32_t *samples, int n) {
  for (int i = 0; i < n - 1; ++i) {
    for (int j = i + 1; j < n; ++j) {
      if (samples[j] < samples[i]) {
        const uint32_t t = samples[i];
        samples[i] = samples[j];
        samples[j] = t;
      }
    }
  }
  return (samples[n / 2 - 1] + samples[n / 2]) / 2;
}

}  // namespace

void BatteryMonitor::read_adc() {
  if (_pin < 0) {
    return;
  }

  uint32_t samples[8];
  for (int i = 0; i < 8; i++) {
    samples[i] = analogReadMilliVolts(_pin);
  }
  const uint32_t mv = median_mv(samples, 8);

  // 1S + 27k/10k @ ~4.0 V cal: GPIO2 usually 360–560 mV. C3 ADC throws wild samples — reject.
  constexpr uint32_t kPinMvMin = 330;
  constexpr uint32_t kPinMvMax = 560;
  if (mv < kPinMvMin || mv > kPinMvMax) {
    return;
  }

  const float v = (mv / 1000.0f) * _scale;
  constexpr float kCellMax = 4.45f;
  if (v < BAT_VOLT_MIN_VALID || v > kCellMax) {
    return;
  }

  if (_primed && _valid && fabsf(v - _volts) > 0.35f) {
    return;
  }

  _pin_mv = (uint16_t)mv;

  if (!_primed) {
    _volts = v;
    _primed = true;
  } else if (_volts > kCellMax) {
    _volts = v;
  } else {
    // Light smoothing — heavy EMA made slow drift look like climbing voltage.
    _volts = 0.5f * _volts + 0.5f * v;
  }

  _valid = _volts >= BAT_VOLT_MIN_VALID;
  if (_valid) {
    _critical = _volts < _shutdown_v;
    _low = _critical || (_volts < _low_v);
  } else {
    _critical = false;
    _low = false;
  }
}

bool BatteryMonitor::service(uint32_t now_ms) {
  if (_pin < 0) {
    return false;
  }
  if (_boot_ms == 0) {
    _boot_ms = now_ms;
  }

  if (_last_adc_ms == 0 || (now_ms - _last_adc_ms) >= BAT_POLL_MS) {
    _last_adc_ms = now_ms;
    read_adc();
  }

#if BAT_SHUTDOWN_ENABLE
  if (!_valid) {
    _below_shutdown_since = 0;
    return false;
  }
  if ((now_ms - _boot_ms) < BAT_STARTUP_GUARD_MS) {
    _below_shutdown_since = 0;
    return false;
  }
  if (_volts < _shutdown_v) {
    if (_below_shutdown_since == 0) {
      _below_shutdown_since = now_ms;
    }
    return (now_ms - _below_shutdown_since) >= _shutdown_hold_ms;
  }
  _below_shutdown_since = 0;
#endif
  return false;
}
