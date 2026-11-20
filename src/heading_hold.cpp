#include "heading_hold.h"

#include <Arduino.h>
#include <math.h>

#include "bot_pins.h"

#ifndef HDG_CAL_MS
#define HDG_CAL_MS 2500u
#endif
#ifndef HDG_CAL_MIN_SAMPLES
#define HDG_CAL_MIN_SAMPLES 50
#endif
#ifndef HDG_CAL_MAX_GZ
#define HDG_CAL_MAX_GZ 0.35f
#endif
#ifndef HDG_LIN_MIN
#define HDG_LIN_MIN 0.06f
#endif

float HeadingHold::wrap_deg(float deg) {
  while (deg > 180.0f) {
    deg -= 360.0f;
  }
  while (deg < -180.0f) {
    deg += 360.0f;
  }
  return deg;
}

void HeadingHold::begin() {
  _cal_start_ms = millis();
  _ready = false;
  _armed = false;
  _yaw_deg = 0.0f;
  _target_deg = 0.0f;
  _bias_gz = 0.0f;
  _bias_n = 0;
  _i_term = 0.0f;
  _last_imu_ms = 0;
}

HeadingHold::Settings HeadingHold::settings() const { return _cfg; }

void HeadingHold::apply_settings(const Settings& s) {
  _cfg = s;
  _enabled = s.enabled;
}

void HeadingHold::set_gains(float kp, float ki, float kd, float max_corr) {
  _cfg.kp = kp;
  _cfg.ki = ki;
  _cfg.kd = kd;
  _cfg.max_corr = max_corr;
}

void HeadingHold::set_enabled(bool on) {
  _cfg.enabled = on;
  _enabled = on;
  if (!on) {
    release();
  }
}

void HeadingHold::update(float gz_rad_s, bool stationary, uint32_t now_ms) {
  _last_gz = gz_rad_s;
  if (_cal_start_ms == 0) {
    _cal_start_ms = now_ms;
  }

  if (!_ready) {
    if (fabsf(gz_rad_s) > HDG_CAL_MAX_GZ) {
      _cal_start_ms = now_ms;
      _bias_n = (_bias_n > 15) ? (_bias_n - 15) : 0;
    } else if (stationary && fabsf(gz_rad_s) < 0.8f) {
      _bias_n++;
      _bias_gz += (gz_rad_s - _bias_gz) / static_cast<float>(_bias_n);
    }
    const uint32_t elapsed = now_ms - _cal_start_ms;
    if (elapsed >= HDG_CAL_MS && _bias_n >= HDG_CAL_MIN_SAMPLES) {
      _ready = true;
      latch();
    }
  } else if (stationary && fabsf(gz_rad_s) < 0.8f) {
    _bias_n++;
    _bias_gz += (gz_rad_s - _bias_gz) / static_cast<float>(_bias_n);
  }

  const float gz = gz_rad_s - _bias_gz;
  if (_last_imu_ms != 0) {
    const uint32_t dt_ms = now_ms - _last_imu_ms;
    if (dt_ms > 0 && dt_ms < 250) {
      _yaw_deg += gz * (57.2957795f * static_cast<float>(dt_ms) * 0.001f);
    }
  }
  _last_imu_ms = now_ms;
}

void HeadingHold::latch() {
  _target_deg = _yaw_deg;
  _armed = true;
  _i_term = 0.0f;
}

void HeadingHold::release() {
  _armed = false;
  _i_term = 0.0f;
}

float HeadingHold::correction(float linear_x, float steer_cmd) {
  if (!_enabled || !_ready || !_armed) {
    return 0.0f;
  }
  if (fabsf(steer_cmd) > 0.02f || fabsf(linear_x) < HDG_LIN_MIN) {
    return 0.0f;
  }

  float err = wrap_deg(_target_deg - _yaw_deg);
  if (fabsf(err) > 35.0f) {
    latch();
    err = 0.0f;
  }

  _i_term += err * 0.02f;
  if (_i_term > 15.0f) {
    _i_term = 15.0f;
  }
  if (_i_term < -15.0f) {
    _i_term = -15.0f;
  }

  const float gz = _last_gz - _bias_gz;
  float w = _cfg.kp * err + _cfg.ki * _i_term + _cfg.kd * (-gz);
  if (w > _cfg.max_corr) {
    w = _cfg.max_corr;
  }
  if (w < -_cfg.max_corr) {
    w = -_cfg.max_corr;
  }
  if (linear_x < -HDG_LIN_MIN) {
    w = -w;
  }
  return w;
}