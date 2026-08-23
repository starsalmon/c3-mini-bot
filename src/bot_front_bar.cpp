#include "bot_front_bar.h"

#include <Adafruit_NeoPixel.h>
#include <math.h>

#include "bot_pins.h"

namespace {

Adafruit_NeoPixel* g_px = nullptr;

uint32_t rgb(uint8_t r, uint8_t g, uint8_t b) {
  return (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) | static_cast<uint32_t>(b);
}

uint8_t clamp_u8(int v) {
  return static_cast<uint8_t>(constrain(v, 0, 255));
}

uint32_t scale_rgb(uint32_t c, float gain) {
  const uint8_t r = static_cast<uint8_t>((c >> 16) & 0xFF);
  const uint8_t g = static_cast<uint8_t>((c >> 8) & 0xFF);
  const uint8_t b = static_cast<uint8_t>(c & 0xFF);
  return rgb(clamp_u8(static_cast<int>(r * gain)), clamp_u8(static_cast<int>(g * gain)),
             clamp_u8(static_cast<int>(b * gain)));
}

void hsv(float h, float s, float v, uint8_t& r, uint8_t& g, uint8_t& b) {
  h = fmodf(h, 1.0f);
  if (h < 0.0f) h += 1.0f;
  const int i = static_cast<int>(floorf(h * 6.0f));
  const float f = h * 6.0f - static_cast<float>(i);
  const float p = v * (1.0f - s);
  const float q = v * (1.0f - f * s);
  const float t = v * (1.0f - (1.0f - f) * s);
  float rf = 0.0f, gf = 0.0f, bf = 0.0f;
  switch (i % 6) {
    case 0: rf = v; gf = t; bf = p; break;
    case 1: rf = q; gf = v; bf = p; break;
    case 2: rf = p; gf = v; bf = t; break;
    case 3: rf = p; gf = q; bf = v; break;
    case 4: rf = t; gf = p; bf = v; break;
    default: rf = v; gf = p; bf = q; break;
  }
  r = static_cast<uint8_t>(rf * 255.0f);
  g = static_cast<uint8_t>(gf * 255.0f);
  b = static_cast<uint8_t>(bf * 255.0f);
}

}  // namespace

bool BotFrontBar::begin(int pin, uint8_t count, uint8_t brightness) {
  _pin = pin;
  _count = count;
  _brightness = brightness;
  _ok = false;

  if (_pin < 0 || _count == 0) {
    return false;
  }
  if (g_px) {
    delete g_px;
    g_px = nullptr;
  }
  g_px = new Adafruit_NeoPixel(_count, _pin, FRONT_BAR_PIXEL_ORDER + NEO_KHZ800);
  if (!g_px) return false;
  g_px->begin();
  g_px->setBrightness(_brightness);
  g_px->clear();
  g_px->show();

  _scan_i = 0;
  _scan_dir = 1;
  _scan_next_ms = 0;
  _spark_next_ms = 0;
  _ok = true;
  return true;
}

void BotFrontBar::advance_scanner(uint32_t now_ms, uint32_t step_ms) {
  if (now_ms < _scan_next_ms) return;
  _scan_next_ms = now_ms + step_ms;
  _scan_i += _scan_dir;
  if (_scan_i >= static_cast<int>(_count) - 1) {
    _scan_i = static_cast<int>(_count) - 1;
    _scan_dir = -1;
  } else if (_scan_i <= 0) {
    _scan_i = 0;
    _scan_dir = 1;
  }
}

void BotFrontBar::render_scanner(uint32_t now_ms, uint32_t base_rgb, uint32_t dot_rgb) {
  if (!g_px) return;

  advance_scanner(now_ms, 55);

  // draw base + trail
  for (uint8_t i = 0; i < _count; i++) {
    float tail = 0.0f;
    const int d = abs(static_cast<int>(i) - _scan_i);
    if (d == 0) {
      tail = 1.0f;
    } else if (d == 1) {
      tail = 0.35f;
    } else if (d == 2) {
      tail = 0.16f;
    } else {
      tail = 0.0f;
    }
    const uint32_t c = (tail > 0.0f) ? scale_rgb(dot_rgb, tail) : base_rgb;
    g_px->setPixelColor(i, c);
  }
  g_px->show();
}

void BotFrontBar::render_rainbow_scanner(uint32_t now_ms, uint32_t dot_rgb) {
  if (!g_px) return;
  advance_scanner(now_ms, 65);
  const float base = fmodf((now_ms / 1000.0f) * 0.10f, 1.0f);
  for (uint8_t i = 0; i < _count; i++) {
    const float h =
        fmodf(base + (static_cast<float>(i) / max(1.0f, static_cast<float>(_count))) * 0.30f, 1.0f);
    uint8_t r = 0, g = 0, b = 0;
    hsv(h, 1.0f, 0.55f, r, g, b);
    uint32_t c = rgb(r, g, b);

    const int d = abs(static_cast<int>(i) - _scan_i);
    if (d == 0) {
      c = dot_rgb;
    } else if (d == 1) {
      c = scale_rgb(dot_rgb, 0.35f);
    } else if (d == 2) {
      c = scale_rgb(dot_rgb, 0.16f);
    }
    g_px->setPixelColor(i, c);
  }
  g_px->show();
}

void BotFrontBar::render_turn(uint32_t now_ms,
                              bool turn_right,
                              uint32_t base_left,
                              uint32_t base_right,
                              uint32_t dot_rgb) {
  if (!g_px) return;
  advance_scanner(now_ms, 55);
  for (uint8_t i = 0; i < _count; i++) {
    const bool right_half = i >= (_count / 2);
    uint32_t base = right_half ? base_right : base_left;
    // Emphasize the side we’re turning toward.
    base = scale_rgb(base, (right_half == turn_right) ? 0.70f : 0.22f);

    const int d = abs(static_cast<int>(i) - _scan_i);
    uint32_t c = base;
    if (d == 0) {
      c = dot_rgb;
    } else if (d == 1) {
      c = scale_rgb(dot_rgb, 0.35f);
    } else if (d == 2) {
      c = scale_rgb(dot_rgb, 0.16f);
    }
    g_px->setPixelColor(i, c);
  }
  g_px->show();
}

void BotFrontBar::render_pulse(uint32_t now_ms, uint32_t rgb_a, uint32_t rgb_b, float hz) {
  if (!g_px) return;
  const float t = now_ms / 1000.0f;
  const float u = 0.5f + 0.5f * sinf(t * 2.0f * static_cast<float>(M_PI) * hz);
  const uint8_t ar = static_cast<uint8_t>((rgb_a >> 16) & 0xFF);
  const uint8_t ag = static_cast<uint8_t>((rgb_a >> 8) & 0xFF);
  const uint8_t ab = static_cast<uint8_t>(rgb_a & 0xFF);
  const uint8_t br = static_cast<uint8_t>((rgb_b >> 16) & 0xFF);
  const uint8_t bg = static_cast<uint8_t>((rgb_b >> 8) & 0xFF);
  const uint8_t bb = static_cast<uint8_t>(rgb_b & 0xFF);
  const uint32_t c = rgb(
      clamp_u8(static_cast<int>(ar + (br - ar) * u)),
      clamp_u8(static_cast<int>(ag + (bg - ag) * u)),
      clamp_u8(static_cast<int>(ab + (bb - ab) * u)));
  for (uint8_t i = 0; i < _count; i++) {
    g_px->setPixelColor(i, c);
  }
  g_px->show();
}

void BotFrontBar::render_rainbow(uint32_t now_ms) {
  if (!g_px) return;
  const float base = fmodf((now_ms / 1000.0f) * 0.10f, 1.0f);
  for (uint8_t i = 0; i < _count; i++) {
    const float h = fmodf(base + (static_cast<float>(i) / max(1.0f, static_cast<float>(_count))) * 0.25f, 1.0f);
    uint8_t r = 0, g = 0, b = 0;
    hsv(h, 1.0f, 0.60f, r, g, b);
    g_px->setPixelColor(i, r, g, b);
  }
  g_px->show();
}

void BotFrontBar::sparkle_overlay(uint32_t now_ms, uint32_t rgb_c, float prob_per_tick) {
  if (!g_px) return;
  if (now_ms < _spark_next_ms) return;
  _spark_next_ms = now_ms + 70;
  if (random(0, 10000) > static_cast<int>(prob_per_tick * 10000.0f)) {
    return;
  }
  const uint8_t i = static_cast<uint8_t>(random(0, _count));
  g_px->setPixelColor(i, rgb_c);
  g_px->show();
}

void BotFrontBar::tick(uint32_t now_ms,
                       bool ros_ok,
                       bool cmd_active,
                       float lin,
                       float ang,
                       bool sonar_ok,
                       float sonar_m,
                       bool ir_detected,
                       bool stalled,
                       bool bump_pulse) {
  if (!_ok || !g_px) return;

  // Default look (boot/idle): rainbow, always.
  // Then override to more “robot readable” patterns in motion / danger.
  if (!ros_ok || !cmd_active) {
    render_rainbow_scanner(now_ms, rgb(140, 140, 140));
    if (ir_detected) {
      sparkle_overlay(now_ms, rgb(0, 180, 60), 0.10f);
    }
    return;
  }

  if (stalled) {
    // "I'm stuck" — loud but power-capped.
    render_pulse(now_ms, rgb(140, 0, 0), rgb(190, 55, 0), 5.0f);
    if (bump_pulse) {
      sparkle_overlay(now_ms, rgb(160, 160, 160), 0.55f);
    }
    if (ir_detected) {
      sparkle_overlay(now_ms, rgb(0, 180, 60), 0.14f);
    }
    return;
  }

  const bool turning = fabsf(ang) > 0.05f;
  const bool reversing = lin < -0.02f;
  const bool close = sonar_ok && isfinite(sonar_m) && (sonar_m > 0.0f) && (sonar_m < 0.30f);

  if (close) {
    // High-signal warning, but cap peak current to avoid brownouts.
    render_pulse(now_ms, rgb(140, 20, 0), rgb(180, 55, 0), 3.2f);
    if (bump_pulse) {
      sparkle_overlay(now_ms, rgb(160, 160, 160), 0.35f);
    }
    if (ir_detected) {
      sparkle_overlay(now_ms, rgb(0, 180, 60), 0.18f);
    }
    return;
  }

  if (reversing) {
    render_scanner(now_ms, rgb(12, 0, 22), rgb(120, 0, 160));
    if (bump_pulse) {
      sparkle_overlay(now_ms, rgb(160, 160, 160), 0.22f);
    }
    if (ir_detected) {
      sparkle_overlay(now_ms, rgb(0, 180, 60), 0.12f);
    }
    return;
  }

  if (turning) {
    const bool turn_right = ang > 0.0f;
    // Direction cue: brighter on the side you're turning toward, with a scanner dot.
    render_turn(now_ms, turn_right, rgb(0, 75, 160), rgb(0, 75, 160), rgb(160, 160, 160));
    if (bump_pulse) {
      sparkle_overlay(now_ms, rgb(160, 160, 160), 0.16f);
    }
    if (ir_detected) {
      sparkle_overlay(now_ms, rgb(0, 180, 60), 0.10f);
    }
    return;
  }

  // Driving default: rainbow with a white scanner.
  render_rainbow_scanner(now_ms, rgb(140, 140, 140));
  if (bump_pulse) {
    sparkle_overlay(now_ms, rgb(160, 160, 160), 0.12f);
  }
  if (ir_detected) {
    sparkle_overlay(now_ms, rgb(0, 180, 60), 0.10f);
  }
}

