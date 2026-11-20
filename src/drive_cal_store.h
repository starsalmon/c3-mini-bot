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

/** Read NVS namespace for diagnostics (no magic check). */
struct NvsSnapshot {
  bool open_ok = false;
  uint32_t magic = 0;
  uint16_t stop_l = 0;
  uint16_t stop_r = 0;
  uint16_t slew = 0;
  uint16_t maxoff = 0;
};
bool read_nvs_snapshot(NvsSnapshot* out);

struct BootCalStatus {
  bool load_ok = false;
  bool nvs_open_fail = false;
  uint32_t magic_read = 0;
  Settings loaded{};
  bool servos_attached = false;
};
void set_boot_cal_status(const BootCalStatus& st);
const BootCalStatus& boot_cal_status();

void apply(DriveServos& drive, const Settings& s);
Settings from_drive(const DriveServos& drive);

}  // namespace DriveCalStore