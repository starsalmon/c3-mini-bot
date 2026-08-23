#pragma once

#include <Arduino.h>
#include <math.h>

#include "bot_pins.h"
#include "mpu6050_imu.h"

/**
 * IMU-derived motion features for the mini-bot.
 *
 * Goals:
 * - Detect "bump/skip" events (jerk spikes) while driving.
 * - Detect "stalled" state when we have motion intent but the body is not moving
 *   (low gyro + low accel deviation) OR we see repeated bump/skip hits.
 *
 * Notes:
 * - "No motion" needs an intent signal (cmd_vel active, or autonomous setpoint).
 * - We intentionally keep the math simple and robust; this is not a full estimator.
 */
class BotImuFeatures {
 public:
  void reset() {
    _last_ms = 0;
    _last_a_mag = NAN;
    _bump_window_start_ms = 0;
    _bump_count = 0;
    _bump_pulse_until_ms = 0;
    _still_since_ms = 0;
    _motion_since_ms = 0;
    _stall_active = false;
    _stall_latch_until_ms = 0;
  }

  // Call once per control tick with your current motion "intent".
  void note_intent(bool motion_intent, uint32_t now_ms) {
    _motion_intent = motion_intent;
    if (!_motion_intent) {
      // When there's no intent, immediately clear (and clear bump accumulation).
      _still_since_ms = 0;
      _motion_since_ms = 0;
      _bump_window_start_ms = 0;
      _bump_count = 0;
      _stall_active = false;
      _stall_latch_until_ms = 0;
      _bump_pulse_until_ms = 0;
    }
    (void)now_ms;
  }

  // Feed each IMU sample. Safe to call at 20ms cadence.
  void note_imu(uint32_t now_ms, const ImuSample& s) {
    const float ax = s.ax;
    const float ay = s.ay;
    const float az = s.az;
    const float gx = s.gx;
    const float gy = s.gy;
    const float gz = s.gz;

    const float a_mag = sqrtf(ax * ax + ay * ay + az * az);
    const float a_dev = fabsf(a_mag - IMU_G_MPS2);
    const float g_mag = sqrtf(gx * gx + gy * gy + gz * gz);

    // --- bump/skip detection (jerk spikes) ---
    if (_last_ms != 0 && isfinite(_last_a_mag)) {
      const float dt = static_cast<float>(now_ms - _last_ms) / 1000.0f;
      if (dt > 0.004f && dt < 0.250f) {
        const float jerk = fabsf(a_mag - _last_a_mag) / dt;  // m/s^3
        if (_motion_intent && jerk >= IMU_BUMP_JERK_T) {
          _bump_pulse_until_ms = now_ms + IMU_BUMP_PULSE_MS;
          bumped_ = true;

          if (_bump_window_start_ms == 0 || (now_ms - _bump_window_start_ms) > STALL_BUMP_WINDOW_MS) {
            _bump_window_start_ms = now_ms;
            _bump_count = 1;
          } else if (_bump_count < 255) {
            _bump_count++;
          }
        }
      }
    }

    _last_ms = now_ms;
    _last_a_mag = a_mag;

    // --- stillness / motion scoring ---
    const bool still = (g_mag <= IMU_STILL_GYRO_RAD_S) && (a_dev <= IMU_STILL_ADEV_MPS2);
    const bool moving = (g_mag >= IMU_MOVE_GYRO_RAD_S) || (a_dev >= IMU_MOVE_ADEV_MPS2);

    if (_motion_intent) {
      if (still) {
        if (_still_since_ms == 0) _still_since_ms = now_ms;
        _motion_since_ms = 0;
      } else if (moving) {
        if (_motion_since_ms == 0) _motion_since_ms = now_ms;
        _still_since_ms = 0;
      } else {
        // grey zone: don't flip state; leave timers as-is
      }

      // --- stall decision ---
      bool trigger = false;

      if (_still_since_ms != 0 && (now_ms - _still_since_ms) >= STALL_STILL_MS) {
        trigger = true;
      }
      if (_bump_window_start_ms != 0 && (now_ms - _bump_window_start_ms) <= STALL_BUMP_WINDOW_MS &&
          _bump_count >= STALL_BUMP_COUNT) {
        trigger = true;
      }

      if (!_stall_active && trigger) {
        _stall_active = true;
        _stall_latch_until_ms = now_ms + STALL_LATCH_MS;
      }

      if (_stall_active) {
        // Hold stall for a minimum time, then clear once we're clearly moving again.
        if (now_ms < _stall_latch_until_ms) {
          // keep latched
        } else if (_motion_since_ms != 0 && (now_ms - _motion_since_ms) >= STALL_CLEAR_MOTION_MS) {
          _stall_active = false;
          _still_since_ms = 0;
          _motion_since_ms = 0;
          _bump_window_start_ms = 0;
          _bump_count = 0;
        }
      }
    }
  }

  // One-tick pulse (for UI). This is not latched; read it each loop.
  bool bump_pulse(uint32_t now_ms) const {
    return _motion_intent && (now_ms < _bump_pulse_until_ms);
  }

  bool stall_active() const { return _stall_active && _motion_intent; }

  // Convenience: detect intent from cmd_vel values.
  static bool intent_from_cmd(float lin, float ang, bool cmd_active) {
    if (!cmd_active) return false;
    return (fabsf(lin) >= STALL_INTENT_LIN) || (fabsf(ang) >= STALL_INTENT_ANG);
  }

  // Deprecated compatibility: a latched "bumped" flag (cleared by tick()).
  bool bumped_ = false;

 private:
  bool _motion_intent = false;

  uint32_t _last_ms = 0;
  float _last_a_mag = NAN;

  // bump accumulation
  uint32_t _bump_window_start_ms = 0;
  uint8_t _bump_count = 0;
  uint32_t _bump_pulse_until_ms = 0;

  // still/motion timers while intent is active
  uint32_t _still_since_ms = 0;
  uint32_t _motion_since_ms = 0;

  // stall latch
  bool _stall_active = false;
  uint32_t _stall_latch_until_ms = 0;
};

