#include "heading_hold_store.h"

#include <math.h>

#include <Preferences.h>

namespace {

constexpr const char* kNs = "hdg_hold";

}  // namespace

namespace HeadingHoldStore {

bool load(NvsSettings* out) {
  if (!out) {
    return false;
  }
  *out = NvsSettings{};
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
  out->enabled = prefs.getUChar("en", 1);
  out->kp = prefs.getFloat("kp", 0.018f);
  out->ki = prefs.getFloat("ki", 0.002f);
  out->kd = prefs.getFloat("kd", 0.10f);
  out->max_corr = prefs.getFloat("max", 0.25f);
  prefs.end();
  return true;
}

bool save(const NvsSettings& in) {
  Preferences prefs;
  if (!prefs.begin(kNs, false)) {
    return false;
  }
  prefs.putUInt("magic", kMagic);
  prefs.putUChar("en", in.enabled ? 1 : 0);
  prefs.putFloat("kp", in.kp);
  prefs.putFloat("ki", in.ki);
  prefs.putFloat("kd", in.kd);
  prefs.putFloat("max", in.max_corr);
  prefs.end();
  NvsSettings verify{};
  return load(&verify) && verify.enabled == in.enabled && fabsf(verify.kp - in.kp) < 0.0001f;
}

void apply(HeadingHold& hdg, const NvsSettings& s) {
  HeadingHold::Settings cfg;
  cfg.enabled = s.enabled != 0;
  cfg.kp = s.kp;
  cfg.ki = s.ki;
  cfg.kd = s.kd;
  cfg.max_corr = s.max_corr;
  hdg.apply_settings(cfg);
}

NvsSettings from_heading_hold(const HeadingHold& hdg) {
  const HeadingHold::Settings cfg = hdg.settings();
  NvsSettings s;
  s.enabled = cfg.enabled ? 1 : 0;
  s.kp = cfg.kp;
  s.ki = cfg.ki;
  s.kd = cfg.kd;
  s.max_corr = cfg.max_corr;
  return s;
}

}  // namespace HeadingHoldStore