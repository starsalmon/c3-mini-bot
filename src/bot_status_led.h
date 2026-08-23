#pragma once

#include <Arduino.h>

enum class BotLedMode {
  Boot,
  Fault,    // red — servo / init fail
  NoSonar,  // purple pulse — HC-SR04 no echo
  Scan,     // blue pulse
  Cruise,   // green
  Reverse,  // orange
  Turn,     // cyan
  LowBatt,  // red pulse — 1S LiPo below critical
};

void bot_led_begin();
void bot_led_set(BotLedMode mode);
void bot_led_tick(uint32_t now_ms);
