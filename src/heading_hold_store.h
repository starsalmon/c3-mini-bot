#pragma once

#include <stdint.h>

#include "heading_hold.h"

namespace HeadingHoldStore {

constexpr uint32_t kMagic = 0xC3D10101u;

struct NvsSettings {
  uint32_t magic = kMagic;
  uint8_t enabled = 1;
  float kp = 0.018f;
  float ki = 0.002f;
  float kd = 0.10f;
  float max_corr = 0.25f;
};

bool load(NvsSettings* out);
bool save(const NvsSettings& in);
void apply(HeadingHold& hdg, const NvsSettings& s);
NvsSettings from_heading_hold(const HeadingHold& hdg);

}  // namespace HeadingHoldStore