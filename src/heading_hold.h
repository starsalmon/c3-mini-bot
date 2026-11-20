#pragma once

#include <stdint.h>

class HeadingHold {
 public:
  void begin();
  void set_gains(float kp, float ki, float kd, float max_corr);
  void set_enabled(bool on);
  bool enabled() const { return _enabled; }
  bool ready() const { return _ready; }
  bool armed() const { return _armed; }
  float yaw_deg() const { return _yaw_deg; }
  float target_deg() const { return _target_deg; }
  float bias_gz() const { return _bias_gz; }
  float last_gz() const { return _last_gz; }
  float integral() const { return _i_term; }

  void update(float gz_rad_s, bool stationary, uint32_t now_ms);
  void latch();
  void release();
  /** Straight-line angular trim; flip sign when reversing (linear_x negative). */
  float correction(float linear_x, float steer_cmd);

  struct Settings {
    bool enabled = true;
    float kp = 0.018f;
    float ki = 0.002f;
    float kd = 0.10f;
    float max_corr = 0.25f;
  };
  Settings settings() const;
  void apply_settings(const Settings& s);

 private:
  static float wrap_deg(float deg);

  Settings _cfg;
  bool _enabled = true;
  bool _ready = false;
  bool _armed = false;
  float _yaw_deg = 0.0f;
  float _target_deg = 0.0f;
  float _bias_gz = 0.0f;
  float _last_gz = 0.0f;
  float _i_term = 0.0f;
  int _bias_n = 0;
  uint32_t _cal_start_ms = 0;
  uint32_t _last_imu_ms = 0;
};