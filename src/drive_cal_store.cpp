#include "drive_cal_store.h"

#include <Preferences.h>

#include "bot_pins.h"
#include "drive_servos.h"

namespace {

constexpr const char* kNs = "drv_cal";

}  // namespace

namespace DriveCalStore {

bool load(Settings* out) {
  if (!out) {
    return false;
  }
  *out = Settings{};
  Preferences prefs;
  if (!prefs.begin(kNs, true)) {
    return false;
  }
  const uint32_t magic = prefs.getUInt("magic", 0);
  if (magic != kMagic) {
    prefs.end();
    return false;
  }
  out->magic = magic;
  out->stop_us_left =
      static_cast<uint16_t>(prefs.getUShort("stop_l", SERVO_STOP_US_LEFT));
  out->stop_us_right =
      static_cast<uint16_t>(prefs.getUShort("stop_r", SERVO_STOP_US_RIGHT));
  out->slew_us_per_10ms = static_cast<uint16_t>(
      prefs.getUShort("slew", SERVO_SLEW_US_PER_10MS));
  out->max_offset_us =
      static_cast<uint16_t>(prefs.getUShort("maxoff", SERVO_MAX_OFFSET_US));
  prefs.end();
  return true;
}

bool save(const Settings& in) {
  Preferences prefs;
  if (!prefs.begin(kNs, false)) {
    return false;
  }
  prefs.putUInt("magic", kMagic);
  prefs.putUShort("stop_l", in.stop_us_left);
  prefs.putUShort("stop_r", in.stop_us_right);
  prefs.putUShort("slew", in.slew_us_per_10ms);
  prefs.putUShort("maxoff", in.max_offset_us);
  prefs.end();
  return true;
}

void apply(DriveServos& drive, const Settings& s) {
  drive.set_stop_us(s.stop_us_left, s.stop_us_right);
  drive.set_motion_limits(s.slew_us_per_10ms, s.max_offset_us);
}

Settings from_drive(const DriveServos& drive) {
  Settings s;
  s.stop_us_left = drive.stop_us_left();
  s.stop_us_right = drive.stop_us_right();
  s.slew_us_per_10ms = drive.slew_us_per_10ms();
  s.max_offset_us = drive.max_offset_us();
  return s;
}

}  // namespace DriveCalStore