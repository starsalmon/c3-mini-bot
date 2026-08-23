#include "bot_power.h"

#include <WiFi.h>
#include <esp_sleep.h>

#include "battery.h"
#include "bot_pins.h"
#include "bot_status_led.h"
#include "drive_servos.h"
#include "ir_tx.h"
#include "pan_servo.h"

namespace {

DriveServos *s_drive = nullptr;
PanServo *s_pan = nullptr;
IrTx *s_ir_tx = nullptr;

}  // namespace

void bot_power_register(DriveServos *drive, PanServo *pan, IrTx *ir_tx) {
  s_drive = drive;
  s_pan = pan;
  s_ir_tx = ir_tx;
}

void bot_halt_low_battery(const BatteryMonitor &battery) {
  Serial.printf("\n*** CELL %.2f V < %.1f — deep sleep (protect unprotected LiPo) ***\n",
                battery.voltage(), BAT_VOLT_SHUTDOWN);
  Serial.flush();

  if (s_ir_tx) {
    s_ir_tx->set_enabled(false);
  }
  if (s_drive) {
    s_drive->stop();
  }
  if (s_pan) {
    s_pan->set_deg(PAN_CENTER_DEG);
  }

  bot_led_set(BotLedMode::LowBatt);
  bot_led_tick(millis());
  delay(50);

#if defined(MICRO_ROS_WIFI)
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
#endif

#if BAT_BUCK_EN_PIN >= 0
  pinMode(BAT_BUCK_EN_PIN, OUTPUT);
  digitalWrite(BAT_BUCK_EN_PIN, LOW);
#endif

  // No wake sources — only USB plug-in / reset button recovers.
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_deep_sleep_start();

  while (true) {
    delay(1000);
  }
}
