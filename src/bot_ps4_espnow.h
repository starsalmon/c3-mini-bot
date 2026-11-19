#pragma once

#include <stdint.h>

struct BotPs4DriveOut {
  bool override_ros;
  bool failsafe;
  float linear_x;
  float angular_z;
};

#if defined(MICRO_ROS_WIFI) && defined(ENABLE_PS4_ESPNOW)

void bot_ps4_espnow_begin();
/** RX + ESP-NOW acks; safe to call without ROS agent. */
void bot_ps4_espnow_tick(uint32_t now_ms);
BotPs4DriveOut bot_ps4_espnow_poll(uint32_t now_ms);
bool bot_ps4_manual_active();

#else

inline void bot_ps4_espnow_begin() {}
inline void bot_ps4_espnow_tick(uint32_t) {}
inline BotPs4DriveOut bot_ps4_espnow_poll(uint32_t) {
  return {};
}
inline bool bot_ps4_manual_active() { return false; }

#endif