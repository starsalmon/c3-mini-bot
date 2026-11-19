#pragma once

#include <Arduino.h>

/** Register WiFi event hooks before WiFi.connect (so GOT_IP is not missed). */
void bot_ota_prepare_wifi_events();
void bot_ota_begin();
void bot_ota_tick();
bool bot_ota_busy();
/** Call when micro-ROS tears down — OTA TCP must re-bind. */
void bot_ota_session_reset();
/** Block up to ms but keep servicing OTA (and servo web when built). */
void bot_ota_delay(uint32_t ms);