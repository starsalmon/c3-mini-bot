#include <Arduino.h>
#include <cmath>
#include <limits>
#include <micro_ros_platformio.h>
#include <WiFi.h>

#include <geometry_msgs/msg/twist.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>

#include "battery.h"
#include "bot_ota.h"
#include "bot_power.h"

#if defined(MICRO_ROS_WIFI)
#include <WiFi.h>
#endif
#include "bot_pins.h"
#include "bot_front_bar.h"
#include "bot_status_led.h"
#include "drive_servos.h"
#include "ir_beacon.h"
#include "ir_tx.h"
#include "bot_imu_features.h"
#include "mpu6050_imu.h"
#include "pan_servo.h"
#include "servo_hw.h"
#include "ultrasonic.h"
#include "oled_ssd1306.h"

namespace {

constexpr uint32_t CMD_TIMEOUT_MS = 400;
constexpr uint32_t RANGE_PERIOD_MS = 100;
constexpr uint32_t IMU_PERIOD_MS = 20;
constexpr uint32_t IR_PERIOD_MS = 30;
constexpr uint32_t LOOP_MS = 20;
constexpr uint32_t AGENT_WAIT_MS = 60000;

DriveServos drive;
BatteryMonitor battery;
PanServo pan_servo;
Ultrasonic sonar;
Mpu6050Imu imu;
OLEDSSD1306 oled;
IrBeacon ir;
IrTx ir_tx;
BotFrontBar front_bar;

bool oled_ok = false;
float last_lin = 0.0f;
float last_ang = 0.0f;
uint32_t last_cmd_ms = 0;
uint32_t drive_arm_at_ms = 0;
bool drive_arm_announced = false;
float pan_deg = PAN_CENTER_DEG;
bool imu_ok = false;
bool sonar_ok = false;
float last_sonar_m = NAN;
bool last_ir_detected = false;
bool stall_active = false;
bool stall_last_pub = false;
uint32_t last_stall_pub_ms = 0;

BotImuFeatures imu_features;

rcl_publisher_t range_pub;
rcl_publisher_t pan_pub;
rcl_publisher_t imu_pub;
rcl_publisher_t ir_detected_pub;
rcl_publisher_t battery_pub;
rcl_publisher_t stall_pub;
rcl_subscription_t cmd_sub;
sensor_msgs__msg__Range range_msg;
std_msgs__msg__Float32 pan_msg;
sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__Bool ir_detected_msg;
std_msgs__msg__Float32 battery_msg;
std_msgs__msg__Bool stall_msg;
geometry_msgs__msg__Twist cmd_msg;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
bool ros_ok = false;
uint8_t ros_init_level = 0;

#define ROS_LOG_FAIL(name, rc) \
  Serial.printf("ROS fail %s: %d\n", (name), static_cast<int>(rc))

void apply_drive() {
  if (drive_arm_at_ms == 0 || (millis() - drive_arm_at_ms) < DRIVE_ARM_MS) {
    drive.stop();
    return;
  }
  if (!drive_arm_announced) {
    drive_arm_announced = true;
    Serial.println("drive armed");
  }
  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
    drive.stop();
    return;
  }
  drive.set_twist(last_lin, last_ang);
}

void on_cmd(const void *msgin) {
  const auto *msg = static_cast<const geometry_msgs__msg__Twist *>(msgin);
  last_lin = static_cast<float>(msg->linear.x);
  last_ang = static_cast<float>(msg->angular.z);
  last_cmd_ms = millis();
}

void sync_ir_tx() {
#if IR_TX_FOLLOW_CMD
  const bool cmd_active = (millis() - last_cmd_ms) <= CMD_TIMEOUT_MS;
  ir_tx.set_enabled(IR_TX_DEFAULT_ON || cmd_active);
#else
  ir_tx.set_enabled(IR_TX_DEFAULT_ON);
#endif
}

void sync_status_led() {
  if (!drive.ok() || !pan_servo.ok()) {
    bot_led_set(BotLedMode::Fault);
    return;
  }
  if (!ros_ok) {
    bot_led_set(BotLedMode::Boot);
    return;
  }
  if (!sonar_ok) {
    bot_led_set(BotLedMode::NoSonar);
    return;
  }
#if IR_PAN_SCAN
  if (!ir.read_any()) {
    bot_led_set(BotLedMode::Scan);
    return;
  }
#endif
  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
    bot_led_set(BotLedMode::Cruise);
    return;
  }
  if (last_lin < -0.05f) {
    bot_led_set(BotLedMode::Reverse);
  } else if (fabsf(last_ang) > 0.08f) {
    bot_led_set(BotLedMode::Turn);
  } else {
    bot_led_set(BotLedMode::Cruise);
  }
}

bool wait_for_agent(uint32_t timeout_ms) {
  const uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    const uint32_t t = millis();
    bot_led_tick(t);
#if defined(ENABLE_OTA)
    bot_ota_tick();
#endif
    if (rmw_uros_ping_agent(200, 3) == RMW_RET_OK) {
      return true;
    }
    Serial.println("waiting for agent...");
    delay(500);
  }
  return false;
}

void destroy_entities();

bool create_entities() {
  allocator = rcl_get_default_allocator();

  rcl_ret_t rc = rclc_support_init(&support, 0, nullptr, &allocator);
  if (rc != RCL_RET_OK) {
    ROS_LOG_FAIL("rclc_support_init", rc);
    return false;
  }
  ros_init_level = 1;

  rc = rclc_node_init_default(&node, BOT_NODE_NAME, BOT_NAMESPACE, &support);
  if (rc != RCL_RET_OK) {
    ROS_LOG_FAIL("rclc_node_init_default", rc);
    destroy_entities();
    return false;
  }
  ros_init_level = 2;

  rc = rclc_publisher_init_default(
      &range_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "sonar/range");
  if (rc != RCL_RET_OK) {
    ROS_LOG_FAIL("pub sonar/range", rc);
    destroy_entities();
    return false;
  }
  rc = rclc_publisher_init_default(
      &pan_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "sonar/pan_deg");
  if (rc != RCL_RET_OK) {
    ROS_LOG_FAIL("pub sonar/pan_deg", rc);
    destroy_entities();
    return false;
  }
  rc = rclc_publisher_init_default(
      &imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu");
  if (rc != RCL_RET_OK) {
    ROS_LOG_FAIL("pub imu", rc);
    destroy_entities();
    return false;
  }
  rc = rclc_publisher_init_default(
      &ir_detected_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "ir/detected");
  if (rc != RCL_RET_OK) {
    ROS_LOG_FAIL("pub ir/detected", rc);
    destroy_entities();
    return false;
  }
  rc = rclc_publisher_init_default(
      &battery_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "battery/voltage");
  if (rc != RCL_RET_OK) {
    ROS_LOG_FAIL("pub battery/voltage", rc);
    destroy_entities();
    return false;
  }
  rc = rclc_publisher_init_default(
      &stall_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "stall");
  if (rc != RCL_RET_OK) {
    ROS_LOG_FAIL("pub stall", rc);
    destroy_entities();
    return false;
  }
  ros_init_level = 3;

  rc = rclc_subscription_init_default(
      &cmd_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
  if (rc != RCL_RET_OK) {
    ROS_LOG_FAIL("sub cmd_vel", rc);
    destroy_entities();
    return false;
  }
  ros_init_level = 4;

  rc = rclc_executor_init(&executor, &support.context, 1, &allocator);
  if (rc != RCL_RET_OK) {
    ROS_LOG_FAIL("rclc_executor_init", rc);
    destroy_entities();
    return false;
  }
  ros_init_level = 5;

  rc = rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &on_cmd, ON_NEW_DATA);
  if (rc != RCL_RET_OK) {
    ROS_LOG_FAIL("executor add cmd_vel", rc);
    destroy_entities();
    return false;
  }
  ros_init_level = 6;

  range_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
  range_msg.field_of_view = 0.26f;
  range_msg.min_range = 0.02f;
  range_msg.max_range = 4.0f;
  pan_msg.data = pan_deg;

  imu_msg.orientation_covariance[0] = -1.0;
  imu_msg.angular_velocity_covariance[0] = 0.02f;
  imu_msg.linear_acceleration_covariance[0] = 0.04f;
  static char imu_frame[] = "imu_link";
  imu_msg.header.frame_id.data = imu_frame;
  imu_msg.header.frame_id.size = sizeof(imu_frame) - 1;

  ros_ok = true;
  return true;
}

void destroy_entities() {
  if (ros_init_level >= 6) {
    rclc_executor_fini(&executor);
  }
  if (ros_init_level >= 4) {
    rcl_subscription_fini(&cmd_sub, &node);
  }
  if (ros_init_level >= 3) {
    rcl_publisher_fini(&stall_pub, &node);
    rcl_publisher_fini(&battery_pub, &node);
    rcl_publisher_fini(&ir_detected_pub, &node);
    rcl_publisher_fini(&imu_pub, &node);
    rcl_publisher_fini(&pan_pub, &node);
    rcl_publisher_fini(&range_pub, &node);
  }
  if (ros_init_level >= 2) {
    rcl_node_fini(&node);
  }
  if (ros_init_level >= 1) {
    rclc_support_fini(&support);
  }
  ros_init_level = 0;
  ros_ok = false;
  drive_arm_at_ms = 0;
  drive_arm_announced = false;
  stall_active = false;
  stall_last_pub = false;
  last_stall_pub_ms = 0;
  imu_features.reset();
}

bool connect_ros() {
  if (!wait_for_agent(AGENT_WAIT_MS)) {
    Serial.println("agent not reachable — check AGENT_IP, UDP 8888, same subnet");
    return false;
  }
  if (!create_entities()) {
    Serial.println("micro-ROS entity init failed — retrying in 2s");
    destroy_entities();
    delay(2000);
    return false;
  }
  // C3 LEDC: 38 kHz IR TX can clobber servo PWM if wheels attached first.
  drive.reattach(SERVO_LEFT_PIN, SERVO_RIGHT_PIN);
  pan_servo.reattach(SERVO_PAN_PIN);
  drive.stop();
  pan_servo.set_deg(pan_deg);
  drive_arm_at_ms = millis();
  drive_arm_announced = false;
  Serial.printf("micro-ROS ready — drive locked %u ms for IMU settle\n", DRIVE_ARM_MS);
  return true;
}

#if IR_PAN_SCAN
static uint32_t random_glance_delay_ms() {
  const uint32_t span = SONAR_GLANCE_MAX_MS - SONAR_GLANCE_MIN_MS;
  return SONAR_GLANCE_MIN_MS + static_cast<uint32_t>(random(static_cast<long>(span + 1)));
}

static float random_glance_offset_deg() {
  const int min_i = static_cast<int>(SONAR_GLANCE_DEG_MIN);
  const int max_i = static_cast<int>(SONAR_GLANCE_DEG_MAX);
  const int mag = random(min_i, max_i + 1);
  return (random(0, 2) == 0) ? -static_cast<float>(mag) : static_cast<float>(mag);
}

void tick_pan_scan(uint32_t now) {
  static int sweep_dir = 1;
  static uint32_t last_step_ms = 0;
  static uint32_t hold_until_ms = 0;
  // Driving sonar glance: 0=wait at center, 1=move to side, 2=hold, 3=return
  static uint8_t glance_state = 0;
  static uint32_t next_glance_ms = 0;
  static uint32_t glance_hold_until_ms = 0;
  static float glance_target_deg = static_cast<float>(PAN_CENTER_DEG);
  static bool was_moving = false;

  if (ir.read_any()) {
    hold_until_ms = now + IR_PAN_HOLD_MS;
    pan_servo.set_deg(pan_deg);
    return;
  }

  // Sonar is on the pan bracket — point forward whenever the body is moving.
  // Use actual ramped wheel state, not only the latest ROS command. A stop
  // command can arrive while the servos are still physically moving.
  const bool body_moving = drive.moving();
  if (!body_moving && was_moving) {
    glance_state = 0;
    next_glance_ms = 0;
    pan_deg = static_cast<float>(PAN_CENTER_DEG);
    pan_servo.set_deg(pan_deg);
  }
  was_moving = body_moving;

  if (body_moving) {
    if (next_glance_ms == 0) {
      next_glance_ms = now + random_glance_delay_ms();
    }

    if (glance_state == 0) {
      pan_deg = static_cast<float>(PAN_CENTER_DEG);
      pan_servo.set_deg(pan_deg);
      if (now >= next_glance_ms) {
        glance_target_deg =
            static_cast<float>(PAN_CENTER_DEG) + random_glance_offset_deg();
        glance_state = 1;
        last_step_ms = now;
      }
      return;
    }

    if (now - last_step_ms < SONAR_GLANCE_STEP_MS) {
      return;
    }
    last_step_ms = now;

    if (glance_state == 1) {
      const float delta = glance_target_deg - pan_deg;
      if (fabsf(delta) <= SONAR_GLANCE_STEP_DEG) {
        pan_deg = glance_target_deg;
        glance_state = 2;
        glance_hold_until_ms = now + SONAR_GLANCE_HOLD_MS;
      } else {
        pan_deg += (delta > 0.0f ? 1.0f : -1.0f) * SONAR_GLANCE_STEP_DEG;
      }
      pan_servo.set_deg(pan_deg);
      return;
    }

    if (glance_state == 2) {
      if (now < glance_hold_until_ms) {
        return;
      }
      glance_state = 3;
      return;
    }

    // glance_state == 3: return to center
    const float center = static_cast<float>(PAN_CENTER_DEG);
    const float delta = center - pan_deg;
    if (fabsf(delta) <= SONAR_GLANCE_STEP_DEG) {
      pan_deg = center;
      pan_servo.set_deg(pan_deg);
      glance_state = 0;
      next_glance_ms = now + random_glance_delay_ms();
      return;
    }
    pan_deg += (delta > 0.0f ? 1.0f : -1.0f) * SONAR_GLANCE_STEP_DEG;
    pan_servo.set_deg(pan_deg);
    return;
  }

  if (now < hold_until_ms) {
    pan_servo.set_deg(pan_deg);
    return;
  }

  // Stopped: sweep for IR beacon.
  if (now - last_step_ms < IR_PAN_SWEEP_MS) {
    return;
  }
  last_step_ms = now;

  pan_deg += static_cast<float>(sweep_dir) * IR_PAN_SWEEP_DEG;
  if (pan_deg >= static_cast<float>(PAN_MAX_DEG)) {
    pan_deg = static_cast<float>(PAN_MAX_DEG);
    sweep_dir = -1;
  } else if (pan_deg <= static_cast<float>(PAN_MIN_DEG)) {
    pan_deg = static_cast<float>(PAN_MIN_DEG);
    sweep_dir = 1;
  }
  pan_servo.set_deg(pan_deg);
}
#endif

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("*** FIRMWARE: c3_mini_wifi (ROS /cmd_vel) ***");
  Serial.println("c3-mini-bot boot");

  bot_led_begin();
  bot_led_set(BotLedMode::Boot);
  front_bar.begin(FRONT_BAR_PIN, FRONT_BAR_COUNT, FRONT_BAR_BRIGHTNESS);

  servo_hw_init();
  battery.begin(PIN_BAT_ADC, BAT_ADC_SCALE, BAT_VOLT_LOW, BAT_VOLT_SHUTDOWN,
                BAT_SHUTDOWN_HOLD_MS);
  bot_power_register(&drive, &pan_servo, &ir_tx);
#if BAT_SHUTDOWN_ENABLE
  Serial.printf("Battery monitor GPIO%d (low-cell shutdown %.2f V enabled)\n", PIN_BAT_ADC,
                BAT_VOLT_SHUTDOWN);
#else
  Serial.printf("Battery monitor GPIO%d (report only, shutdown off)\n", PIN_BAT_ADC);
#endif

#if defined(MICRO_ROS_WIFI)
#ifndef WIFI_SSID
#define WIFI_SSID "changeme"
#define WIFI_PSK "changeme"
#define AGENT_IP "192.168.1.1"
#ifndef AGENT_PORT
#define AGENT_PORT 8888
#endif
#endif
#if defined(ENABLE_OTA)
#ifndef BOT_OTA_HOSTNAME
#define BOT_OTA_HOSTNAME "c3-mini-bot"
#endif
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(BOT_OTA_HOSTNAME);
#endif
  IPAddress agent_ip;
  if (!agent_ip.fromString(AGENT_IP)) {
    Serial.printf("WARN: AGENT_IP parse failed: '%s'\n", AGENT_IP);
    agent_ip = IPAddress(192, 168, 1, 1);
  }
  set_microros_wifi_transports(WIFI_SSID, WIFI_PSK, agent_ip, AGENT_PORT);
  WiFi.setSleep(WIFI_PS_NONE);
  delay(3000);
  Serial.printf("WiFi → agent %s:%d\n", AGENT_IP, AGENT_PORT);
  Serial.print("ESP IP: ");
  Serial.println(WiFi.localIP());
#if defined(ENABLE_OTA)
  bot_ota_begin();
#endif
#else
  Serial.println("micro-ROS serial transport (USB)");
#endif

  sonar.begin(SONAR_TRIG_PIN, SONAR_ECHO_PIN);
  ir.begin(IR_RECV_LEFT_PIN, IR_RECV_RIGHT_PIN);
  if (ir_tx.begin(IR_TX_PIN, IR_TX_HZ)) {
    sync_ir_tx();
    Serial.printf("IR TX %u Hz GPIO %d (follows cmd_vel=%d)\n", IR_TX_HZ, IR_TX_PIN,
                  IR_TX_FOLLOW_CMD);
  } else {
    Serial.println("WARN: IR TX PWM init failed");
  }
  if (!drive.begin(SERVO_LEFT_PIN, SERVO_RIGHT_PIN)) {
    Serial.println("WARN: wheel servos not attached");
  } else {
    drive.stop();
  }
  pan_servo.begin(SERVO_PAN_PIN);
  pan_servo.set_deg(pan_deg);
  imu_ok = imu.begin(I2C_SDA_PIN, I2C_SCL_PIN, MPU6050_ADDR);
  if (!imu_ok) {
    Serial.println("WARN: MPU6050 not found on I2C");
  } else {
    Serial.println("MPU6050 OK");
  }
  imu_features.reset();

  oled_ok = oled.begin();

  Serial.printf("micro-ROS ns=/%s node=%s\n", BOT_NAMESPACE, BOT_NODE_NAME);
  // ROS connect runs in loop() so OTA can accept uploads during agent wait.
}

void loop() {
  static uint32_t last_ping = 0;
  static uint32_t last_range = 0;
  static uint32_t last_imu = 0;
  static uint32_t last_ir = 0;
  static uint32_t last_bat = 0;
  const uint32_t now = millis();

  if (battery.service(now)) {
    bot_halt_low_battery(battery);
  }

#if defined(ENABLE_OTA)
  bot_ota_tick();
  if (bot_ota_busy()) {
    drive.stop();
    front_bar.tick(now, ros_ok, false, 0.0f, 0.0f, sonar_ok, last_sonar_m, last_ir_detected, false, false);
    bot_led_tick(now);
    delay(10);
    return;
  }
#endif

  if (!ros_ok) {
    connect_ros();
    drive.stop();
    sync_status_led();
    front_bar.tick(now, ros_ok, false, 0.0f, 0.0f, sonar_ok, last_sonar_m, last_ir_detected, false, false);
    bot_led_tick(millis());
    delay(200);
    return;
  }

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  if (now - last_ping > 1000) {
    last_ping = now;
    if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
      Serial.println("agent lost");
      destroy_entities();
      ir_tx.set_enabled(false);
      drive.stop();
      bot_led_set(BotLedMode::Fault);
      return;
    }
  }

  apply_drive();
  drive.tick();
  sync_ir_tx();
  ir_tx.tick();

#if IR_PAN_SCAN
  tick_pan_scan(now);
#endif

  if (now - last_range >= RANGE_PERIOD_MS) {
    last_range = now;
    const float m = sonar.read_range_m();
    if (m > 0.0f) {
      sonar_ok = true;
    }
    last_sonar_m = m;
    range_msg.range = (m < 0.0f) ? std::numeric_limits<float>::quiet_NaN() : m;
    range_msg.header.stamp.sec = static_cast<int32_t>(now / 1000);
    range_msg.header.stamp.nanosec = static_cast<uint32_t>((now % 1000) * 1000000UL);
    if (rcl_publish(&range_pub, &range_msg, nullptr) != RCL_RET_OK) {
      Serial.println("publish failed");
    }
    pan_msg.data = pan_deg;
    rcl_publish(&pan_pub, &pan_msg, nullptr);
  }

  if (imu_ok && now - last_imu >= IMU_PERIOD_MS) {
    last_imu = now;
    ImuSample sample;
    if (imu.read(sample)) {
      const bool cmd_active = (now - last_cmd_ms) <= CMD_TIMEOUT_MS;
      const bool motion_intent = BotImuFeatures::intent_from_cmd(last_lin, last_ang, cmd_active);
      imu_features.note_intent(motion_intent, now);
      imu_features.note_imu(now, sample);
      stall_active = imu_features.stall_active();

      imu_msg.header.stamp.sec = static_cast<int32_t>(now / 1000);
      imu_msg.header.stamp.nanosec = static_cast<uint32_t>((now % 1000) * 1000000UL);
      imu_msg.linear_acceleration.x = sample.ax;
      imu_msg.linear_acceleration.y = sample.ay;
      imu_msg.linear_acceleration.z = sample.az;
      imu_msg.angular_velocity.x = sample.gx;
      imu_msg.angular_velocity.y = sample.gy;
      imu_msg.angular_velocity.z = sample.gz;
      rcl_publish(&imu_pub, &imu_msg, nullptr);

      // Publish stall state on change (and occasionally refresh).
      if (stall_active != stall_last_pub || (now - last_stall_pub_ms) > 1500) {
        stall_last_pub = stall_active;
        last_stall_pub_ms = now;
        stall_msg.data = stall_active;
        rcl_publish(&stall_pub, &stall_msg, nullptr);
      }
    }
  }

  if (now - last_ir >= IR_PERIOD_MS) {
    last_ir = now;
    ir_detected_msg.data = ir.read_any();
    last_ir_detected = bool(ir_detected_msg.data);
    rcl_publish(&ir_detected_pub, &ir_detected_msg, nullptr);
  }

  if (now - last_bat >= BAT_PERIOD_MS) {
    last_bat = now;
    battery_msg.data = battery.voltage();
    rcl_publish(&battery_pub, &battery_msg, nullptr);
  }

  sync_status_led();
  const bool cmd_active = (now - last_cmd_ms) <= CMD_TIMEOUT_MS;
  const bool bump_pulse = imu_features.bump_pulse(now);
  front_bar.tick(now, ros_ok, cmd_active, last_lin, last_ang, sonar_ok, last_sonar_m, last_ir_detected,
                 stall_active, bump_pulse);
  bot_led_tick(now);
  delay(LOOP_MS);
}
