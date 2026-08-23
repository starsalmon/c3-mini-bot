#include "ir_tx.h"

namespace {

constexpr uint8_t kDutyBits = 8;
constexpr uint8_t kDutyMax = (1u << kDutyBits) - 1u;

}  // namespace

bool IrTx::begin(int pin, uint32_t hz) {
  _pin = pin;
  _duty_on = kDutyMax / 2;
  const uint8_t p = static_cast<uint8_t>(_pin);
  // LEDC ch5 (timer 2) — GPIO 0/1/3 servos use ch0–2.
  if (!ledcAttachChannel(p, hz, kDutyBits, 5)) {
    ledcDetach(p);
    if (!ledcAttachChannel(p, hz, kDutyBits, 5)) {
      _ok = false;
      return false;
    }
  }
  ledcWrite(p, 0);
  _ok = true;
  _armed = false;
  return true;
}

void IrTx::apply_carrier(bool on) {
  ledcWrite(static_cast<uint8_t>(_pin), on ? _duty_on : 0);
}

void IrTx::set_enabled(bool on) {
  if (!_ok) {
    return;
  }
  _armed = on;
#if !IR_TX_MODULATE
  apply_carrier(on);
#else
  if (!on) {
    apply_carrier(false);
  }
#endif
}

void IrTx::tick() {
  if (!_ok || !_armed) {
    return;
  }
#if IR_TX_MODULATE
  const uint32_t period = IR_TX_BURST_US + IR_TX_GAP_US;
  const bool burst = (micros() % period) < IR_TX_BURST_US;
  apply_carrier(burst);
#else
  apply_carrier(true);
#endif
}
