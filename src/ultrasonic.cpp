#include "ultrasonic.h"

#include "bot_pins.h"

namespace {

constexpr uint32_t kMinIntervalMs = 60;
constexpr uint32_t kMinPulseUs = 116;
constexpr uint32_t kMaxPulseUs = 25000;
// round-trip µs → metres (343 m/s): pulse / 1e6 * 343 / 2
constexpr float kUsToMetres = 343.0f / 2.0f / 1000000.0f;

void sort_u32(uint32_t* a, uint8_t n) {
  for (uint8_t i = 1; i < n; ++i) {
    const uint32_t v = a[i];
    uint8_t j = i;
    while (j > 0 && a[j - 1] > v) {
      a[j] = a[j - 1];
      --j;
    }
    a[j] = v;
  }
}

}  // namespace

bool Ultrasonic::begin(int trig_pin, int echo_pin) {
  _trig = trig_pin;
  _echo = echo_pin;
  pinMode(_trig, OUTPUT);
  // Some HC-SR04 clones can float when unplugged; keep echo pulled down.
  pinMode(_echo, INPUT_PULLDOWN);
  digitalWrite(_trig, LOW);
  delay(60);
  return true;
}

uint32_t Ultrasonic::ping_once(uint32_t timeout_us) {
  if (_trig < 0 || _echo < 0) return 0;

  const uint32_t now_ms = millis();
  if (_last_ping_ms != 0) {
    const uint32_t elapsed = now_ms - _last_ping_ms;
    if (elapsed < kMinIntervalMs) {
      delay(kMinIntervalMs - elapsed);
    }
  }
  _last_ping_ms = millis();

  digitalWrite(_trig, LOW);
  delayMicroseconds(2);
  digitalWrite(_trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trig, LOW);

  // Avoid Arduino pulseIn() here: on ESP32 it may use RMT internally, which can
  // conflict with NeoPixel RMT usage (front bar / status LED).
  const uint32_t start_wait = micros();
  // Wait for any previous HIGH to end.
  while (digitalRead(_echo) == HIGH) {
    if (static_cast<uint32_t>(micros() - start_wait) >= timeout_us) {
      return 0;
    }
  }
  // Wait for rising edge.
  while (digitalRead(_echo) == LOW) {
    if (static_cast<uint32_t>(micros() - start_wait) >= timeout_us) {
      return 0;
    }
  }
  const uint32_t pulse_start = micros();
  // Wait for falling edge.
  while (digitalRead(_echo) == HIGH) {
    if (static_cast<uint32_t>(micros() - pulse_start) >= timeout_us) {
      return 0;
    }
  }
  const uint32_t pulse = static_cast<uint32_t>(micros() - pulse_start);
  if (pulse < kMinPulseUs || pulse > kMaxPulseUs) {
    return 0;
  }
  return pulse;
}

float Ultrasonic::read_range_m(uint32_t timeout_us) {
  if (_trig < 0 || _echo < 0) return -1.0f;

  uint32_t samples[1];
  uint8_t count = 0;
  for (int i = 0; i < 1; ++i) {
    const uint32_t pulse = ping_once(timeout_us);
    if (pulse > 0) {
      samples[count++] = pulse;
    }
  }

  if (count == 0) {
    _last_pulse_us = 0;
    return -1.0f;
  }

  sort_u32(samples, count);
  const uint32_t pulse = samples[count / 2];
  _last_pulse_us = pulse;

  return static_cast<float>(pulse) * kUsToMetres;
}

void Ultrasonic::diagnose(Stream& out) {
  if (_trig < 0 || _echo < 0) {
    out.println("sonar: not initialized");
    return;
  }

  out.printf("sonar TRIG=GPIO%d ECHO=GPIO%d\n", _trig, _echo);
  out.printf("  echo idle=%d (expect 0)\n", digitalRead(_echo));
  out.println("  flat surface ~30cm away, 3 pings:");

  for (int i = 0; i < 3; ++i) {
    const uint32_t pulse = ping_once(30000);
    if (pulse == 0) {
      out.printf("  %d: TIMEOUT\n", i + 1);
    } else {
      out.printf("  %d: %lu us → %.0f cm\n", i + 1, pulse,
                 static_cast<float>(pulse) / 58.0f);
    }
    delay(80);
  }
}
