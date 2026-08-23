#pragma once

// Waveshare ESP32-C3 Zero — safe GPIO picks (avoid 4=JTAG/flash strap, 8–9=boot I2C labels).
// GPIO10 = onboard WS2812 (status). GPIO20/21 = USB-UART console.

#ifndef SERVO_LEFT_PIN
#define SERVO_LEFT_PIN 0   // FS90R wheel L (signal)
#endif
#ifndef SERVO_RIGHT_PIN
#define SERVO_RIGHT_PIN 1  // FS90R wheel R (signal)
#endif
#ifndef SERVO_PAN_PIN
#define SERVO_PAN_PIN 3    // FS90 pan (ultrasonic mount)
#endif
#ifndef SONAR_TRIG_PIN
#define SONAR_TRIG_PIN 5
#endif
#ifndef SONAR_ECHO_PIN
#define SONAR_ECHO_PIN 6   // 5 V echo → divider to ~3.3 V (e.g. 1k→GPIO, 1.7k→GND)
#endif
#ifndef STATUS_LED_PIN
#define STATUS_LED_PIN 10
#endif

// Front NeoPixel bar (8 pixels) — user add-on.
// Default: GPIO19 (free on C3 Zero; avoid 4, 8/9 I2C, 10 onboard pixel, 20/21 UART).
#ifndef FRONT_BAR_PIN
#define FRONT_BAR_PIN 19
#endif
#ifndef FRONT_BAR_COUNT
#define FRONT_BAR_COUNT 8
#endif
#ifndef FRONT_BAR_BRIGHTNESS
// Keep conservative by default (servos + WiFi + pixels can brownout small 5V regs).
#define FRONT_BAR_BRIGHTNESS 28
#endif

// Front bar NeoPixel color order.
// Many strips are GRB; some are RGB (rainbow looks "off" if mismatched).
// Try switching to NEO_RGB if reds/greens look swapped.
#ifndef FRONT_BAR_PIXEL_ORDER
#define FRONT_BAR_PIXEL_ORDER NEO_RGB
#endif

// FS90R continuous rotation: 1500 µs = stop
#ifndef SERVO_STOP_US
#define SERVO_STOP_US 1500
#endif
#ifndef SERVO_MIN_US
#define SERVO_MIN_US 1100
#endif
#ifndef SERVO_MAX_US
#define SERVO_MAX_US 1900
#endif

// Set to 1 if that wheel servo is physically mounted reversed (one side mirrored).
#ifndef SERVO_INVERT_LEFT
#define SERVO_INVERT_LEFT 0
#endif
#ifndef SERVO_INVERT_RIGHT
#define SERVO_INVERT_RIGHT 1
#endif
// Per-wheel speed scale at bench (1.0 = nominal). If bot arcs right on straight B 30, try SERVO_TRIM_RIGHT 0.96.
#ifndef SERVO_TRIM_LEFT
#define SERVO_TRIM_LEFT 1.00f
#endif
#ifndef SERVO_TRIM_RIGHT
#define SERVO_TRIM_RIGHT 0.96f
#endif

#ifndef PAN_CENTER_DEG
#define PAN_CENTER_DEG 90
#endif
#ifndef PAN_MIN_DEG
#define PAN_MIN_DEG 30
#endif
#ifndef PAN_MAX_DEG
#define PAN_MAX_DEG 150
#endif

// micro-ROS fleet: unique per physical bot (bot1, bot2, …). Topics become /bot1/cmd_vel etc.
#ifndef BOT_NAMESPACE
#define BOT_NAMESPACE "bot1"
#endif
#ifndef BOT_NODE_NAME
#define BOT_NODE_NAME "mini_bot"
#endif

// I2C — Waveshare SDA/SCL labels (GPIO8/9).
#ifndef I2C_SDA_PIN
#define I2C_SDA_PIN 8
#endif
#ifndef I2C_SCL_PIN
#define I2C_SCL_PIN 9
#endif
#ifndef MPU6050_ADDR
#define MPU6050_ADDR 0x68
#endif

// TSOP4138 — LOW = 38 kHz IR seen. Use -1 to disable a channel.
#ifndef IR_RECV_LEFT_PIN
#define IR_RECV_LEFT_PIN -1
#endif
#ifndef IR_RECV_RIGHT_PIN
#define IR_RECV_RIGHT_PIN 7
#endif
// Single TSOP on the pan head: sweep servo when stopped, hold bearing when seen.
#ifndef IR_PAN_SCAN
#define IR_PAN_SCAN 1
#endif
#ifndef IR_PAN_SWEEP_MS
#define IR_PAN_SWEEP_MS 120
#endif
#ifndef IR_PAN_SWEEP_DEG
#define IR_PAN_SWEEP_DEG 2.5f
#endif

// Wheel ramp per drive.tick() at 50 Hz — smooth accel/decel (tune on bench, not via ROS).
#ifndef DRIVE_MAX_STEP
#define DRIVE_MAX_STEP 0.04f
#endif
#ifndef DRIVE_SPIN_STEP
#define DRIVE_SPIN_STEP 0.03f
#endif
#ifndef DRIVE_MOVE_EPS
#define DRIVE_MOVE_EPS 0.03f
#endif
// After ROS connect: hold wheels stopped while IMU/brain calibrate (servo twitch settle).
#ifndef DRIVE_ARM_MS
#define DRIVE_ARM_MS 3500
#endif
#ifndef SONAR_GLANCE_MIN_MS
#define SONAR_GLANCE_MIN_MS 500
#endif
#ifndef SONAR_GLANCE_MAX_MS
#define SONAR_GLANCE_MAX_MS 3000
#endif
#ifndef SONAR_GLANCE_DEG_MIN
#define SONAR_GLANCE_DEG_MIN 14.0f
#endif
#ifndef SONAR_GLANCE_DEG_MAX
#define SONAR_GLANCE_DEG_MAX 28.0f
#endif
#ifndef SONAR_GLANCE_HOLD_MS
#define SONAR_GLANCE_HOLD_MS 280
#endif
#ifndef SONAR_GLANCE_STEP_MS
#define SONAR_GLANCE_STEP_MS 100
#endif
#ifndef SONAR_GLANCE_STEP_DEG
#define SONAR_GLANCE_STEP_DEG 3.0f
#endif
#ifndef IR_PAN_HOLD_MS
#define IR_PAN_HOLD_MS 1400
#endif

// IR LED beacon — 38 kHz PWM out. GPIO → resistor → LED cathode/anode → GND.
// For several LEDs, use an NPN transistor to sink current from 5 V.
#ifndef IR_TX_PIN
#define IR_TX_PIN 20
#endif
#ifndef IR_TX_HZ
#define IR_TX_HZ 38000
#endif
#ifndef IR_TX_DEFAULT_ON
#define IR_TX_DEFAULT_ON 0
#endif
// When 1, IR LED turns on while /cmd_vel is active (no extra ROS subscription).
// Keep 0 for swarm — rover front IR mistakes 38 kHz for an obstacle.
#ifndef IR_TX_FOLLOW_CMD
#define IR_TX_FOLLOW_CMD 0
#endif
#ifndef IR_TX_MODULATE
#define IR_TX_MODULATE 1
#endif
#ifndef IR_TX_BURST_US
#define IR_TX_BURST_US 560
#endif
#ifndef IR_TX_GAP_US
#define IR_TX_GAP_US 560
#endif

// Battery sense — calibrated pack voltage, with a sustained low-cell cutoff.
// BAT+ → R1 → GPIO2 → R2 → GND.  DMM cal: 3.36 V cell, 0.89 V tap → ratio 3.78.
#ifndef PIN_BAT_ADC
#define PIN_BAT_ADC 2
#endif
#ifndef BAT_ADC_SCALE
// USB bench cal 2026-08-11: DMM cell 4.00 V, tap 1.06 V, GPIO2 ~476 mV → 4.00/0.476 ≈ 8.40
// C3 ADC drifts a few % on USB power — re-check on battery/WiFi before trusting low-voltage cutoff.
#define BAT_ADC_SCALE 8.40f
#endif
#ifndef BAT_VOLT_MIN_VALID
#define BAT_VOLT_MIN_VALID 1.5f
#endif
#ifndef BAT_SHUTDOWN_ENABLE
#define BAT_SHUTDOWN_ENABLE 0
#endif
#ifndef BAT_VOLT_LOW
#define BAT_VOLT_LOW 3.5f
#endif
#ifndef BAT_VOLT_SHUTDOWN
#define BAT_VOLT_SHUTDOWN 2.9f
#endif
#ifndef BAT_SHUTDOWN_HOLD_MS
#define BAT_SHUTDOWN_HOLD_MS 10000
#endif
#ifndef BAT_STARTUP_GUARD_MS
#define BAT_STARTUP_GUARD_MS 5000
#endif
#ifndef BAT_POLL_MS
#define BAT_POLL_MS 250
#endif
#ifndef BAT_PERIOD_MS
#define BAT_PERIOD_MS 1000
#endif
#ifndef BAT_BUCK_EN_PIN
#define BAT_BUCK_EN_PIN -1
#endif

// ---------------- IMU motion / stall features ----------------
// IMU-derived stall detection is only meaningful when the firmware has motion intent
// (cmd_vel active in micro-ROS, or non-zero setpoint in autonomous).
//
// Two detection paths:
// - "stillness stall": sustained low gyro + low accel deviation while intent is active
// - "bump/skip stall": repeated IMU jerk spikes while intent is active (wheels skipping/bumping)

#ifndef IMU_G_MPS2
#define IMU_G_MPS2 9.80665f
#endif

// What counts as "we intended to move" (Twist units: m/s and rad/s).
#ifndef STALL_INTENT_LIN
#define STALL_INTENT_LIN 0.10f
#endif
#ifndef STALL_INTENT_ANG
#define STALL_INTENT_ANG 0.15f
#endif

// Bump/skip detection: jerk = |Δ|a|| / Δt  (m/s^3)
#ifndef IMU_BUMP_JERK_T
#define IMU_BUMP_JERK_T 140.0f
#endif
#ifndef IMU_BUMP_PULSE_MS
#define IMU_BUMP_PULSE_MS 140
#endif

// Stillness thresholds (tune after a couple runs).
#ifndef IMU_STILL_GYRO_RAD_S
#define IMU_STILL_GYRO_RAD_S 0.18f
#endif
#ifndef IMU_STILL_ADEV_MPS2
#define IMU_STILL_ADEV_MPS2 0.80f
#endif

// Motion thresholds (used to clear a latched stall).
#ifndef IMU_MOVE_GYRO_RAD_S
#define IMU_MOVE_GYRO_RAD_S 0.35f
#endif
#ifndef IMU_MOVE_ADEV_MPS2
#define IMU_MOVE_ADEV_MPS2 1.35f
#endif

// Stall logic timing.
#ifndef STALL_STILL_MS
#define STALL_STILL_MS 750
#endif
#ifndef STALL_BUMP_WINDOW_MS
#define STALL_BUMP_WINDOW_MS 900
#endif
#ifndef STALL_BUMP_COUNT
#define STALL_BUMP_COUNT 4
#endif
#ifndef STALL_LATCH_MS
#define STALL_LATCH_MS 1400
#endif
#ifndef STALL_CLEAR_MOTION_MS
#define STALL_CLEAR_MOTION_MS 250
#endif
