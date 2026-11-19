#pragma once

#include <stdint.h>

struct BotPs4DriveOut {
  bool override_ros;
  bool failsafe;
  float linear_x;
  float angular_z;
};

struct BotPs4LinkStatus {
  bool espnow_ready = false;
  bool linked = false;
  bool manual = false;
  bool ctrl_ok = false;
  bool speed_low = false;
  bool steer_low = false;
  uint8_t channel = 0;
  uint32_t last_pkt_age_ms = 0;
  uint32_t ack_tx = 0;
};

#if defined(MICRO_ROS_WIFI) && defined(ENABLE_PS4_ESPNOW)

void bot_ps4_espnow_begin();
void bot_ps4_espnow_tick(uint32_t now_ms);
BotPs4DriveOut bot_ps4_espnow_poll(uint32_t now_ms);
bool bot_ps4_manual_active();
void bot_ps4_link_status(BotPs4LinkStatus* out);

#else

inline void bot_ps4_espnow_begin() {}
inline void bot_ps4_espnow_tick(uint32_t) {}
inline BotPs4DriveOut bot_ps4_espnow_poll(uint32_t) {
  return {};
}
inline bool bot_ps4_manual_active() { return false; }
inline void bot_ps4_link_status(BotPs4LinkStatus* out) {
  if (out) {
    *out = {};
  }
}

#endif