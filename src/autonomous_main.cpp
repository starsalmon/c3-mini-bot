#include <Arduino.h>
#include <math.h>

#include "bot_pins.h"
#include "bot_front_bar.h"
#include "bot_status_led.h"
#include "drive_servos.h"
#include "bot_imu_features.h"
#include "mpu6050_imu.h"
#include "pan_servo.h"
#include "ultrasonic.h"

namespace {

constexpr float STOP_M = 0.22f;
constexpr float SLOW_M = 0.40f;
constexpr float CRUISE_LIN = 0.30f;
constexpr float REVERSE_LIN = -0.28f;
constexpr float STEER = 0.38f;
constexpr uint32_t PAN_SETTLE_MS = 250;
constexpr uint32_t REVERSE_MS = 700;
constexpr uint32_t TURN_MS = 550;
constexpr uint32_t SCAN_COOLDOWN_MS = 4000;
constexpr uint32_t FORWARD_READ_MS = 250;
constexpr uint32_t LOOP_MS = 20;
constexpr float WIGGLE_AMPLITUDE_DEG = 12.0f;  // ±12° from centre while cruising
constexpr uint32_t WIGGLE_PERIOD_MS = 2800;    // full left-right cycle

enum class Mode { Cruise, Reverse, Turn, Scan };

DriveServos drive;
PanServo pan;
Ultrasonic sonar;
BotFrontBar front_bar;
Mpu6050Imu imu;
bool imu_ok = false;
BotImuFeatures imu_features;
bool stalled = false;

Mode mode = Mode::Cruise;
uint32_t mode_until = 0;
uint32_t last_scan_ms = 0;
uint32_t last_forward_ms = 0;
float scan_best_deg = PAN_CENTER_DEG;
float scan_best_m = 0.0f;
float ahead_m = -1.0f;
bool sonar_ok = false;
uint8_t close_hits = 0;
uint32_t last_imu_ms = 0;
ImuSample last_imu_sample;

void sync_led() {
  if (!drive.ok() || !pan.ok()) {
    bot_led_set(BotLedMode::Fault);
    return;
  }
  if (!sonar_ok) {
    bot_led_set(BotLedMode::NoSonar);
    return;
  }
  switch (mode) {
    case Mode::Cruise:
      bot_led_set(BotLedMode::Cruise);
      break;
    case Mode::Reverse:
      bot_led_set(BotLedMode::Reverse);
      break;
    case Mode::Turn:
      bot_led_set(BotLedMode::Turn);
      break;
    case Mode::Scan:
      bot_led_set(BotLedMode::Scan);
      break;
  }
}

// Ping at whatever angle the pan is currently at (wiggle or centre).
float read_forward_m() {
  const float m = sonar.read_range_m();
  if (m > 0.0f) {
    sonar_ok = true;
  }
  return m;
}

void update_pan_wiggle(uint32_t now_ms) {
  const float phase =
      static_cast<float>(now_ms % WIGGLE_PERIOD_MS) / static_cast<float>(WIGGLE_PERIOD_MS);
  const float deg = static_cast<float>(PAN_CENTER_DEG) +
                    WIGGLE_AMPLITUDE_DEG *
                        sinf(phase * 2.0f * static_cast<float>(M_PI));
  pan.set_deg(deg);
}

void center_pan() {
  pan.set_deg(static_cast<float>(PAN_CENTER_DEG));
}

void run_scan() {
  const Mode prev = mode;
  mode = Mode::Scan;
  sync_led();
  drive.stop();

  static const float kAngles[] = {PAN_MIN_DEG, 60.0f, 90.0f, 120.0f, PAN_MAX_DEG};
  scan_best_m = -1.0f;
  scan_best_deg = PAN_CENTER_DEG;

  for (float deg : kAngles) {
    pan.set_deg(deg);
    delay(PAN_SETTLE_MS);
    const float m = sonar.read_range_m();
    if (m > 0.0f) {
      sonar_ok = true;
      if (m > scan_best_m) {
        scan_best_m = m;
        scan_best_deg = deg;
      }
    }
    Serial.printf("scan %.0f° → %.0f cm (pulse=%lu us)\n", deg,
                  m > 0 ? m * 100.0f : -1.0f, sonar.last_pulse_us());
  }

  pan.set_deg(PAN_CENTER_DEG);
  delay(PAN_SETTLE_MS);
  last_scan_ms = millis();
  last_forward_ms = last_scan_ms;
  ahead_m = read_forward_m();

  Serial.printf("best %.0f° (%.0f cm)\n", scan_best_deg,
                scan_best_m > 0 ? scan_best_m * 100.0f : -1.0f);

  if (!sonar_ok) {
    Serial.println("sonar: no echo — cruising blind (purple LED)");
  }

  mode = prev;
  sync_led();
}

float steer_toward_best() {
  const float err = scan_best_deg - static_cast<float>(PAN_CENTER_DEG);
  if (err < -8.0f) return STEER;
  if (err > 8.0f) return -STEER;
  return 0.0f;
}

void enter_reverse() {
  close_hits = 0;
  center_pan();
  drive.set_twist(REVERSE_LIN, 0.0f);
  mode = Mode::Reverse;
  mode_until = millis() + REVERSE_MS;
  sync_led();
  Serial.println("→ reverse");
}

void enter_turn() {
  center_pan();
  const float ang = steer_toward_best();
  drive.set_twist(0.0f, ang);
  mode = Mode::Turn;
  mode_until = millis() + TURN_MS;
  sync_led();
  Serial.printf("→ turn ang=%.2f\n", ang);
}

void enter_cruise() {
  mode = Mode::Cruise;
  mode_until = 0;
  sync_led();
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("c3-mini-bot autonomous (no ROS)");

  bot_led_begin();
  bot_led_set(BotLedMode::Boot);
  front_bar.begin(FRONT_BAR_PIN, FRONT_BAR_COUNT, FRONT_BAR_BRIGHTNESS);

  const bool drive_ok = drive.begin(SERVO_LEFT_PIN, SERVO_RIGHT_PIN);
  const bool pan_ok = pan.begin(SERVO_PAN_PIN);
  sonar.begin(SONAR_TRIG_PIN, SONAR_ECHO_PIN);
  imu_ok = imu.begin(I2C_SDA_PIN, I2C_SCL_PIN, MPU6050_ADDR);
  if (!imu_ok) {
    Serial.println("WARN: MPU6050 not found on I2C (stall detect disabled)");
  } else {
    Serial.println("MPU6050 OK");
  }
  imu_features.reset();

  if (!drive_ok) {
    Serial.println("ERROR: wheel servo PWM failed (check LEDC / pins 0,1)");
  }
  if (!pan_ok) {
    Serial.println("ERROR: pan servo PWM failed (pin 3)");
  }
  if (!drive_ok || !pan_ok) {
    bot_led_set(BotLedMode::Fault);
    while (true) {
      bot_led_tick(millis());
      delay(20);
    }
  }

  run_scan();
  enter_cruise();
}

void loop() {
  const uint32_t now = millis();

  // IMU features (stall/bump) based on our own setpoints.
  // "Intent" is whatever we're currently commanding to wheels.
  if (imu_ok && (now - last_imu_ms) >= 20) {
    last_imu_ms = now;
    ImuSample s;
    if (imu.read(s)) {
      last_imu_sample = s;
      const bool intent = (mode == Mode::Cruise) || (mode == Mode::Reverse) || (mode == Mode::Turn);
      imu_features.note_intent(intent, now);
      imu_features.note_imu(now, s);
      stalled = imu_features.stall_active();
    }
  }

  if (!drive.ok() || !pan.ok()) {
    bot_led_tick(now);
    front_bar.tick(now, true, false, 0.0f, 0.0f, sonar_ok, ahead_m, false, false, false);
    drive.stop();
    bot_led_set(BotLedMode::Fault);
    delay(100);
    return;
  }

  if (mode == Mode::Reverse || mode == Mode::Turn) {
    bot_led_tick(now);
    const float lin = (mode == Mode::Reverse) ? REVERSE_LIN : 0.0f;
    const float ang = (mode == Mode::Turn) ? steer_toward_best() : 0.0f;
    front_bar.tick(now, true, true, lin, ang, sonar_ok, ahead_m, false, stalled, imu_features.bump_pulse(now));
    if (now >= mode_until) {
      if (mode == Mode::Reverse) {
        enter_turn();
      } else {
        enter_cruise();
      }
    }
    drive.tick();
    delay(LOOP_MS);
    return;
  }

  // Cruise — gentle pan wiggle for a wider view while wheels keep moving.
  update_pan_wiggle(now);

  if (now - last_forward_ms >= FORWARD_READ_MS) {
    last_forward_ms = now;
    ahead_m = read_forward_m();
  }

  if (sonar_ok && ahead_m > 0.0f && ahead_m < STOP_M) {
    if (++close_hits >= 2) {
      Serial.printf("blocked %.0f cm\n", ahead_m * 100.0f);
      run_scan();
      enter_reverse();
      return;
    }
  } else {
    close_hits = 0;
  }

  // Sonar can miss low obstacles (or thin legs) — if IMU says we're stalled, recover.
  if (stalled && mode == Mode::Cruise) {
    Serial.println("stall (IMU) → reverse");
    run_scan();
    enter_reverse();
    return;
  }

  if (now - last_scan_ms >= SCAN_COOLDOWN_MS) {
    run_scan();
  }

  float lin = CRUISE_LIN;
  float ang = steer_toward_best() * 0.25f;
  if (sonar_ok && ahead_m > 0.0f && ahead_m < SLOW_M) {
    lin = CRUISE_LIN * 0.5f;
    ang = STEER * 0.35f;
  }

  drive.set_twist(lin, ang);
  drive.tick();
  front_bar.tick(now, true, true, lin, ang, sonar_ok, ahead_m, false, stalled, imu_features.bump_pulse(now));
  bot_led_tick(now);
  delay(LOOP_MS);
}
