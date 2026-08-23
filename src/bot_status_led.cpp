#include "bot_status_led.h"

#include <Adafruit_NeoPixel.h>
#include <math.h>

#include "bot_pins.h"

namespace {

Adafruit_NeoPixel s_px(1, STATUS_LED_PIN, NEO_RGB + NEO_KHZ800);
BotLedMode s_mode = BotLedMode::Boot;

void write_rgb(uint8_t r, uint8_t g, uint8_t b) {
  s_px.setPixelColor(0, r, g, b);
  s_px.show();
}

uint8_t scale(uint8_t v, float gain) {
  const int x = static_cast<int>(static_cast<float>(v) * gain);
  return static_cast<uint8_t>(constrain(x, 0, 255));
}

}  // namespace

void bot_led_begin() {
  s_px.begin();
  s_px.setBrightness(40);
  s_px.clear();
  s_px.show();
  write_rgb(30, 30, 10);
}

void bot_led_set(BotLedMode mode) {
  s_mode = mode;
}

void bot_led_tick(uint32_t now_ms) {
  const float t = now_ms / 1000.0f;
  const float pulse = 0.5f + 0.5f * sinf(t * 2.0f * static_cast<float>(M_PI) * 1.5f);
  const float fast = 0.5f + 0.5f * sinf(t * 2.0f * static_cast<float>(M_PI) * 4.0f);

  switch (s_mode) {
    case BotLedMode::Boot:
      write_rgb(scale(200, pulse), scale(120, pulse), 0);
      break;
    case BotLedMode::Fault:
      write_rgb(220, 0, 0);
      break;
    case BotLedMode::NoSonar:
      write_rgb(scale(120, fast), 0, scale(180, fast));
      break;
    case BotLedMode::Scan:
      write_rgb(0, scale(80, pulse), scale(220, pulse));
      break;
    case BotLedMode::Cruise:
      write_rgb(0, scale(200, 0.6f + 0.4f * pulse), 0);
      break;
    case BotLedMode::Reverse:
      write_rgb(scale(220, 0.7f + 0.3f * fast), scale(90, 0.7f), 0);
      break;
    case BotLedMode::Turn:
      write_rgb(0, scale(180, fast), scale(200, fast));
      break;
    case BotLedMode::LowBatt:
      write_rgb(scale(240, fast), 0, 0);
      break;
  }
}
