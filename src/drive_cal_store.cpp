#include "drive_cal_store.h"

#include <Preferences.h>

#include "bot_pins.h"
#include "drive_servos.h"

namespace {

constexpr const char* kNs = "drv_cal";
DriveCalStore::BootCalStatus s_boot{};

}  // namespace

namespace DriveCalStore {

void set_boot_cal_status(const BootCalStatus& st) { s_boot = st; }

const BootCalStatus& boot_cal_status() { return s_boot; }

bool read_nvs_snapshot(NvsSnapshot* out) {
  if (!out) {
    return false;
  }
  *out = NvsSnapshot{};
  Preferences prefs;
  if (!prefs.begin(kNs, true)) {
    return false;
  }
  out->open_ok = true;
  out->magic = prefs.getUInt("magic", 0);
  out->stop_l = prefs.getUShort("stop_l", 0);
  out->stop_r = prefs.getUShort("stop_r", 0);
  out->slew = prefs.getUShort("slew", 0);
  out->maxoff = prefs.getUShort("maxoff", 0);
  prefs.end();
  return true;
}

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
  Settings verify;
  if (!load(&verify)) {
    return false;
  }
  return verify.stop_us_left == in.stop_us_left && verify.stop_us_right == in.stop_us_right &&
         verify.slew_us_per_10ms == in.slew_us_per_10ms &&
         verify.max_offset_us == in.max_offset_us;
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