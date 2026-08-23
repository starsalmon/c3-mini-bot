#include <Arduino.h>
#include <Wire.h>

#include "battery.h"
#include "bot_pins.h"
#include "bot_status_led.h"
#include "drive_cli.h"
#include "drive_servos.h"
#include "ir_beacon.h"
#include "ir_tx.h"
#include "mpu6050_imu.h"
#include "pan_servo.h"
#include "servo_hw.h"
#include "ultrasonic.h"

DriveServos drive;
PanServo pan;
Ultrasonic sonar;
Mpu6050Imu imu;
IrBeacon ir;
IrTx ir_tx;
BatteryMonitor battery;

void print_batt_reading() {
  battery.service(millis());
  const uint16_t mv = battery.pin_millivolts();
  const uint16_t raw = analogRead(PIN_BAT_ADC);
  const float cell_now = (mv / 1000.0f) * BAT_ADC_SCALE;
  Serial.printf("GPIO%d: %u mV  raw=%u  cell=%.2f V (scale %.2f)\n", PIN_BAT_ADC, mv, raw,
                cell_now, BAT_ADC_SCALE);
  Serial.println("DMM on pack → scale = cell_V / (pin_mV/1000)");
}

void stream_batt(uint32_t ms) {
  Serial.println("battery stream (mV, cell V):");
  const uint32_t until = millis() + ms;
  while (millis() < until) {
    print_batt_reading();
    delay(500);
  }
}

void print_help() {
  Serial.println();
  Serial.println("=== c3-mini-bot bench (no ROS) ===");
  Serial.println("Drive:  L/R/B <percent>   S=stop   ? = drive help");
  Serial.println("        (percent -100..100, e.g.  L 5   B 30   B -10)");
  Serial.println("Pan:    p then 30-150 + Enter");
  Serial.println("Sonar:  g=one ping   G=stream 5s");
  Serial.println("IMU:    i=I2C scan+init   m=one sample   M=stream 5s");
  Serial.println("IR:     t=TSOP stream 5s   T=toggle IR LED TX (loopback test)");
  Serial.printf("Batt:   v=one reading   V=stream 5s (GPIO%d divider)\n", PIN_BAT_ADC);
  Serial.println("        ?=help   a=status report");
  Serial.println();
  Serial.printf("Pins: L=%d R=%d pan=%d trig=%d echo=%d IR=%d SDA=%d SCL=%d TX=%d BAT=%d\n",
                SERVO_LEFT_PIN, SERVO_RIGHT_PIN, SERVO_PAN_PIN, SONAR_TRIG_PIN,
                SONAR_ECHO_PIN, IR_RECV_RIGHT_PIN, I2C_SDA_PIN, I2C_SCL_PIN, IR_TX_PIN,
                PIN_BAT_ADC);
  Serial.printf("Invert flags: LEFT=%d RIGHT=%d\n", SERVO_INVERT_LEFT, SERVO_INVERT_RIGHT);
}

void i2c_scan() {
  Serial.printf("I2C scan SDA=%d SCL=%d:\n", I2C_SDA_PIN, I2C_SCL_PIN);
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  0x%02X\n", addr);
      ++found;
    }
  }
  if (found == 0) {
    Serial.println("  (no devices — check 3.3V, SDA/SCL, GND)");
  }
}

void report_status() {
  Serial.println("--- hardware status ---");
  Serial.printf("drive: %s\n", drive.ok() ? "OK" : "FAIL");
  Serial.printf("pan:   %s\n", pan.ok() ? "OK" : "FAIL");
  if (imu.ok()) {
    Serial.printf("imu:   OK addr=0x%02X\n", imu.addr());
  } else {
    Serial.println("imu:   not found");
  }
  Serial.printf("ir rx: %s\n", ir.read_any() ? "signal" : "none");
  Serial.printf("ir tx: %s\n", ir_tx.enabled() ? "on" : "off");
  const float m = sonar.read_range_m();
  Serial.printf("sonar: %.0f cm\n", m > 0 ? m * 100.0f : -1.0f);
  print_batt_reading();
  Serial.println("-----------------------");
}

bool init_imu() {
  Wire.end();
  delay(20);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  delay(20);
  i2c_scan();
  const bool ok = imu.begin(I2C_SDA_PIN, I2C_SCL_PIN, MPU6050_ADDR);
  if (ok) {
    Serial.printf("MPU6050 OK @ 0x%02X\n", imu.addr());
    // Amber = bench mode idle (not fleet "cruise" — that misled volt/drive tests).
    bot_led_set(BotLedMode::Boot);
  } else {
    Serial.println("MPU6050 init failed");
    bot_led_set(BotLedMode::Fault);
  }
  return ok;
}

void stream_imu(uint32_t ms) {
  if (!imu.ok()) {
    Serial.println("IMU not ready — press i first");
    return;
  }
  Serial.println("IMU stream (ax ay az m/s^2, gx gy gz rad/s):");
  const uint32_t until = millis() + ms;
  while (millis() < until) {
    ImuSample s;
    if (imu.read(s)) {
      Serial.printf("  %.2f %.2f %.2f | %.3f %.3f %.3f\n", s.ax, s.ay, s.az, s.gx, s.gy,
                    s.gz);
    } else {
      Serial.println("  read fail");
    }
    delay(200);
  }
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println();
  Serial.println("*** FIRMWARE: c3_bench (USB only — no WiFi, no cmd_vel) ***");
  Serial.println("*** Reflash c3_mini_wifi_ota for fleet / ROS drive ***");
  Serial.println();
  bot_led_begin();
  bot_led_set(BotLedMode::Boot);

  servo_hw_init();
  if (!drive.begin(SERVO_LEFT_PIN, SERVO_RIGHT_PIN)) {
    Serial.println("WARN: wheel servos not attached");
  }
  pan.begin(SERVO_PAN_PIN);
  pan.set_deg(PAN_CENTER_DEG);
  sonar.begin(SONAR_TRIG_PIN, SONAR_ECHO_PIN);
  ir.begin(IR_RECV_LEFT_PIN, IR_RECV_RIGHT_PIN);
  if (ir_tx.begin(IR_TX_PIN, IR_TX_HZ)) {
    ir_tx.set_enabled(false);
  }
  battery.begin(PIN_BAT_ADC, BAT_ADC_SCALE, BAT_VOLT_LOW, BAT_VOLT_SHUTDOWN,
                BAT_SHUTDOWN_HOLD_MS);

  init_imu();
  print_help();
  report_status();
}

void loop() {
  static char line[32];
  static uint8_t line_len = 0;

  drive.tick();
  bot_led_tick(millis());
  ir_tx.tick();

  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c != '\n') {
      if (line_len + 1 < sizeof(line)) {
        line[line_len++] = c;
      }
      continue;
    }
    line[line_len] = '\0';
    line_len = 0;
    if (drive_cli_handle_line(drive, Serial, line)) {
      continue;
    }
    if (line[0] == '\0') continue;
    const char key = line[0];
    switch (key) {
    case '?':
      print_help();
      break;
    case 'a':
      report_status();
      break;
    case 'G': {
      Serial.println("sonar stream 5s:");
      drive.stop();
      const uint32_t until = millis() + 5000;
      while (millis() < until) {
        const float m = sonar.read_range_m();
        Serial.printf("  %.0f cm\n", m > 0 ? m * 100.0f : -1.0f);
        delay(200);
      }
      break;
    }
    case 'g': {
      const float m = sonar.read_range_m();
      if (m < 0.0f) {
        Serial.printf("sonar: no echo (pulse_us=%lu)\n", sonar.last_pulse_us());
      } else {
        Serial.printf("sonar: %.0f cm\n", m * 100.0f);
      }
      break;
    }
    case 'i':
      init_imu();
      break;
    case 'm': {
      ImuSample s;
      if (imu.read(s)) {
        Serial.printf("IMU ax=%.2f ay=%.2f az=%.2f gx=%.3f gy=%.3f gz=%.3f\n", s.ax, s.ay,
                      s.az, s.gx, s.gy, s.gz);
      } else {
        Serial.println("IMU read failed — press i");
      }
      break;
    }
    case 'M':
      stream_imu(5000);
      break;
    case 't': {
      Serial.printf("IR GPIO%d — point TV remote or enable IR TX (T), 5s:\n", IR_RECV_RIGHT_PIN);
      Serial.println("  raw=0 means detected (active LOW)");
      const uint32_t until = millis() + 5000;
      uint32_t hits = 0;
      while (millis() < until) {
        const int raw = digitalRead(IR_RECV_RIGHT_PIN);
        if (raw == LOW) {
          ++hits;
        }
        Serial.printf("  raw=%d %s\n", raw, raw == LOW ? "DETECTED" : "none");
        delay(100);
      }
      Serial.printf("detected %lu/50 samples\n", static_cast<unsigned long>(hits));
      break;
    }
    case 'T':
      ir_tx.set_enabled(!ir_tx.enabled());
      Serial.printf("IR TX GPIO%d: %s\n", IR_TX_PIN, ir_tx.enabled() ? "ON" : "off");
      break;
    case 'v':
      print_batt_reading();
      break;
    case 'V':
      stream_batt(5000);
      break;
    case 'p':
      Serial.println("pan: type degrees 30-150 on next line");
      break;
    default:
      break;
    }
  }
}
