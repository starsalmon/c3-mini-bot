#include "bot_ota.h"

#ifndef BOT_OTA_HOSTNAME
#define BOT_OTA_HOSTNAME "c3-mini-bot"
#endif

#if defined(ENABLE_OTA) && defined(MICRO_ROS_WIFI)

#include <ArduinoOTA.h>
#include <WiFi.h>

#if defined(ENABLE_SERVO_WEB)
void bot_servo_web_tick();
void bot_servo_web_on_wifi_event(WiFiEvent_t event);
void bot_servo_web_session_reset();
#else
static void bot_servo_web_tick() {}
static void bot_servo_web_on_wifi_event(WiFiEvent_t) {}
static void bot_servo_web_session_reset() {}
#endif

namespace {

bool s_ota_ok = false;
bool s_ota_busy = false;
bool s_ota_started = false;
bool s_wifi_events = false;
TaskHandle_t s_ota_task = nullptr;

void service_idle() {
  bot_servo_web_tick();
  yield();
}

void start_ota_server() {
  if (WiFi.status() != WL_CONNECTED) {
    s_ota_ok = false;
    return;
  }

  if (s_ota_started) {
    ArduinoOTA.end();
    s_ota_started = false;
    s_ota_ok = false;
  }

  WiFi.setSleep(WIFI_PS_NONE);
  WiFi.setHostname(BOT_OTA_HOSTNAME);
  ArduinoOTA.setHostname(BOT_OTA_HOSTNAME);
#if defined(BOT_OTA_PASS)
  ArduinoOTA.setPassword(BOT_OTA_PASS);
#endif
  ArduinoOTA.onStart([]() {
    s_ota_busy = true;
    Serial.println("OTA start");
  });
  ArduinoOTA.onEnd([]() {
    s_ota_busy = false;
    Serial.println("OTA end — rebooting");
  });
  ArduinoOTA.onError([](ota_error_t err) {
    s_ota_busy = false;
    Serial.printf("OTA err %u\n", err);
    s_ota_ok = false;
    s_ota_started = false;
  });

  ArduinoOTA.begin();
  s_ota_started = true;
  s_ota_ok = true;
  Serial.printf("OTA ready — %s.local  IP %s:3232\n", BOT_OTA_HOSTNAME,
                WiFi.localIP().toString().c_str());
}

void on_wifi_event(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      s_ota_ok = false;
      s_ota_started = false;
      bot_servo_web_on_wifi_event(event);
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      s_ota_ok = false;
      s_ota_started = false;
      bot_servo_web_on_wifi_event(event);
      break;
    default:
      break;
  }
}

void ensure_wifi_events() {
  if (s_wifi_events) {
    return;
  }
  WiFi.onEvent(on_wifi_event);
  s_wifi_events = true;
}

void ota_task_fn(void*) {
  for (;;) {
    bot_ota_tick();
    bot_servo_web_tick();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void ensure_ota_task() {
  if (s_ota_task != nullptr) {
    return;
  }
  xTaskCreate(ota_task_fn, "ota", 4096, nullptr, 1, &s_ota_task);
}

}  // namespace

void bot_ota_begin() {
  ensure_wifi_events();
  ensure_ota_task();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("OTA: WiFi not ready — will retry in loop/task");
    return;
  }
  start_ota_server();
}

void bot_ota_tick() {
  if (WiFi.status() != WL_CONNECTED) {
    s_ota_ok = false;
    s_ota_started = false;
    return;
  }
  if (!s_ota_ok) {
    start_ota_server();
  }
  if (s_ota_ok) {
    ArduinoOTA.handle();
  }
}

void bot_ota_session_reset() {
  s_ota_ok = false;
  s_ota_started = false;
  if (WiFi.status() == WL_CONNECTED) {
    start_ota_server();
  }
  bot_servo_web_session_reset();
}

void bot_ota_delay(uint32_t ms) {
  const uint32_t end = millis() + ms;
  while (static_cast<int32_t>(end - millis()) > 0) {
    bot_ota_tick();
    service_idle();
    delay(10);
  }
}

bool bot_ota_busy() {
  return s_ota_busy;
}

#else

void bot_ota_begin() {}
void bot_ota_tick() {}
void bot_ota_session_reset() {}
void bot_ota_delay(uint32_t ms) {
  delay(ms);
}
bool bot_ota_busy() {
  return false;
}

#endif