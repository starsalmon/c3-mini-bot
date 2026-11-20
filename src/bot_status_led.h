#pragma once

#include <Arduino.h>

// Onboard WS2812 (Waveshare ESP32-C3 Zero): GPIO STATUS_LED_PIN, NEO_RGB order.

struct BotStatusLedInput {
  bool wifi_connected = false;
  bool uros_transport = false;       // Wi-Fi up and XRCE transport configured (serial: transport only)
  bool agent_connected = false;      // micro-ROS session + entities OK
  bool agent_reconnecting = false;   // session create/teardown or agent wait after prior session
};

class BotStatusLed {
 public:
  void begin(int pin);
  void set_input(const BotStatusLedInput& in) { _in = in; }
  void tick(uint32_t now_ms);
  /** One-shot colour (e.g. low-battery shutdown); bypasses phase machine. */
  void show_rgb(uint8_t r, uint8_t g, uint8_t b);

 private:
  enum class Phase : uint8_t {
    kBoot,
    kWifiConnecting,
    kWifiNoAgent,
    kAgentOk,
    kAgentLost,
  };

  static constexpr uint32_t kBootMs = 350;
  static constexpr uint32_t kNetworkGraceMs = 45000;
  static constexpr uint8_t kBrightness = 20;

  int _pin = -1;
  BotStatusLedInput _in;
  uint32_t _boot_until = 0;
  uint32_t _network_grace_until = 0;
  bool _ready = false;
  Phase _last_phase = Phase::kBoot;
  bool _last_on = false;
  uint32_t _last_show_ms = 0;

  Phase _phase(uint32_t now_ms) const;
  void _render(Phase phase, uint32_t now_ms);
  void _show(uint8_t r, uint8_t g, uint8_t b);
};

void bot_led_begin();
void bot_led_set_input(const BotStatusLedInput& in);
void bot_led_tick(uint32_t now_ms);
void bot_led_show_rgb(uint8_t r, uint8_t g, uint8_t b);