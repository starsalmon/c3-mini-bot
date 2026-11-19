#pragma once

#include <WiFi.h>

class DriveServos;

#if defined(ENABLE_SERVO_WEB) && defined(MICRO_ROS_WIFI)

void bot_servo_web_begin(DriveServos& drive);
void bot_servo_web_tick();
void bot_servo_web_on_wifi_event(WiFiEvent_t event);
void bot_servo_web_session_reset();

#else

inline void bot_servo_web_begin(DriveServos&) {}
inline void bot_servo_web_tick() {}
inline void bot_servo_web_on_wifi_event(WiFiEvent_t) {}
inline void bot_servo_web_session_reset() {}

#endif