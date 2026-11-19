#pragma once

#include <stdint.h>

#include "bot_pins.h"

class DriveServos;

namespace DriveCalStore {

constexpr uint32_t kMagic = 0xC3D1CE01u;

struct Settings {
  uint32_t magic = kMagic;
  uint16_t stop_us_left = SERVO_STOP_US_LEFT;
  uint16_t stop_us_right = SERVO_STOP_US_RIGHT;
  uint16_t slew_us_per_10ms = SERVO_SLEW_US_PER_10MS;
  uint16_t max_offset_us = SERVO_MAX_OFFSET_US;
};

bool load(Settings* out);
bool save(const Settings& in);
void apply(DriveServos& drive, const Settings& s);
Settings from_drive(const DriveServos& drive);

}  // namespace DriveCalStore