#pragma once

class DriveServos;

#if defined(ENABLE_SERVO_WEB) && defined(MICRO_ROS_WIFI)

void bot_servo_web_begin(DriveServos& drive);
void bot_servo_web_tick();

#else

inline void bot_servo_web_begin(DriveServos&) {}
inline void bot_servo_web_tick() {}

#endif