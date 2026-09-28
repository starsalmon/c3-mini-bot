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
  read_adc();
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

float pick_1s_scale(uint32_t mv) {
  const float tap = mv / 1000.0f;
  constexpr float kCellMin = 3.15f;
  constexpr float kCellMax = 4.40f;
  const float candidates[] = {2.00f, 3.20f, 3.70f, 8.40f};
  if (mv < 700) {
    const float v = tap * 8.40f;
    if (v >= kCellMin && v <= kCellMax) {
      return 8.40f;
    }
  }
  for (float s : candidates) {
    const float v = tap * s;
    if (v >= kCellMin && v <= kCellMax) {
      return s;
    }
  }
  return 0.0f;
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
  _pin_mv = (uint16_t)mv;

  // C3 ADC 11 dB ~3.1 V full-scale. Old breadboard tap was ~0.5 V; PCB dividers
  // are often ~1.5–2 V. Reject only rails/opens, not the old 330–560 mV window.
  constexpr uint32_t kPinMvMin = 80;
  constexpr uint32_t kPinMvMax = 3100;
  if (mv < kPinMvMin || mv > kPinMvMax) {
    return;
  }

  if (_scale <= 0.1f) {
    const float picked = pick_1s_scale(mv);
    if (picked <= 0.1f) {
      return;
    }
    _scale = picked;
    Serial.printf("Battery auto scale %.2f (tap %u mV)\n", _scale, (unsigned)mv);
  }

  const float v = (mv / 1000.0f) * _scale;
  constexpr float kCellMax = 4.45f;
  if (v < BAT_VOLT_MIN_VALID || v > kCellMax) {
    return;
  }

  if (_primed && _valid && fabsf(v - _volts) > 0.35f) {
    return;
  }

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
