#pragma once

#include <Arduino.h>

/** 8× NeoPixel bar on the bot front — fun UI + motion cues. */
class BotFrontBar {
 public:
  bool begin(int pin, uint8_t count, uint8_t brightness);

  void tick(uint32_t now_ms,
            bool ros_ok,
            bool cmd_active,
            float lin,
            float ang,
            bool sonar_ok,
            float sonar_m,
            bool ir_detected,
            bool stalled,
            bool bump_pulse);

 private:
  void advance_scanner(uint32_t now_ms, uint32_t step_ms);
  void render_scanner(uint32_t now_ms, uint32_t base_rgb, uint32_t dot_rgb);
  void render_rainbow_scanner(uint32_t now_ms, uint32_t dot_rgb);
  void render_turn(uint32_t now_ms, bool turn_right, uint32_t base_left, uint32_t base_right, uint32_t dot_rgb);
  void render_pulse(uint32_t now_ms, uint32_t rgb_a, uint32_t rgb_b, float hz);
  void render_rainbow(uint32_t now_ms);
  void sparkle_overlay(uint32_t now_ms, uint32_t rgb, float prob_per_tick);

  int _pin = -1;
  uint8_t _count = 0;
  uint8_t _brightness = 40;
  bool _ok = false;

  // scanner
  int _scan_i = 0;
  int _scan_dir = 1;
  uint32_t _scan_next_ms = 0;

  // sparkle
  uint32_t _spark_next_ms = 0;
};

