#include "bot_ps4_espnow.h"

#if defined(MICRO_ROS_WIFI) && defined(ENABLE_PS4_ESPNOW)

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include <string.h>

#include "bot_pins.h"
#include "fleet_ps4_drive.h"
#include "fleet_ps4_pkt.h"

#ifndef BOT_PS4_MAX_LIN
#define BOT_PS4_MAX_LIN 0.22f
#endif

#ifndef BOT_PS4_MAX_ANG
#define BOT_PS4_MAX_ANG 0.42f
#endif

#ifndef BOT_PS4_PAN_SLEW_DEG_PER_10MS
#define BOT_PS4_PAN_SLEW_DEG_PER_10MS 2.5f
#endif

static const uint8_t kBridgeMac[6] = {FLEET_PS4_BRIDGE_MAC};

struct RxSlot {
  FleetPs4TxPkt pkt;
  volatile bool fresh;
};

static portMUX_TYPE s_rx_mux = portMUX_INITIALIZER_UNLOCKED;
static RxSlot s_rx;
static uint8_t s_peer_ch = 0;
static bool s_espnow_on = false;
static bool s_manual = false;
static bool s_ctrl_ok = false;
static bool s_speed_low = true;
static bool s_steer_low = false;
static bool s_pending_rumble = false;
static uint32_t s_last_pkt_ms = 0;
static uint32_t s_last_edge_ms = 0;
static int8_t s_lx = 0;
static int8_t s_rx_axis = 0;
static uint8_t s_l2 = 0;
static uint8_t s_r2 = 0;
static uint16_t s_buttons_prev = 0;
static bool s_opt_prev = false;
static volatile uint32_t s_pending_ack_seq = 0;
static BotPs4DriveOut s_drive_out;
static float s_throttle_now = 0.0f;
static uint32_t s_ack_tx = 0;
static float s_pan_deg = static_cast<float>(PAN_CENTER_DEG);
static bool s_pan_active = false;
static uint32_t s_pan_last_ms = 0;

static bool macEq(const uint8_t* a, const uint8_t* b) { return memcmp(a, b, 6) == 0; }

static void sendAck(uint32_t seq) {
  FleetPs4AckPkt ack{};
  ack.channel = (uint8_t)WiFi.channel();
  ack.seq = seq;
  WiFi.macAddress(ack.mac);
  ack.feedback = 0;
  if (s_speed_low) {
    ack.feedback |= FLEET_PS4_ACK_FB_SPEED_LOW;
  }
  if (s_steer_low) {
    ack.feedback |= FLEET_PS4_ACK_FB_STEER_LOW;
  }
  if (s_pending_rumble) {
    ack.feedback |= FLEET_PS4_ACK_FB_RUMBLE;
    s_pending_rumble = false;
  }
  fleetPs4AckSeal(ack);
  if (esp_now_send(kBridgeMac, reinterpret_cast<uint8_t*>(&ack), sizeof(ack)) == ESP_OK) {
    s_ack_tx++;
  }
}

static void IRAM_ATTR onEspNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (!info || len != (int)sizeof(FleetPs4TxPkt)) {
    return;
  }
  if (!macEq(info->src_addr, kBridgeMac)) {
    return;
  }
  FleetPs4TxPkt pkt;
  memcpy(&pkt, data, sizeof(pkt));
  if (!fleetPs4TxValid(pkt)) {
    return;
  }
  portENTER_CRITICAL_ISR(&s_rx_mux);
  s_rx.pkt = pkt;
  s_rx.fresh = true;
  s_pending_ack_seq = pkt.seq;
  portEXIT_CRITICAL_ISR(&s_rx_mux);
}

static bool ensurePeer() {
  if (!s_espnow_on) {
    if (esp_now_init() != ESP_OK) {
      Serial.println(F("PS4: ESP-NOW init failed"));
      return false;
    }
    esp_now_register_recv_cb(onEspNowRecv);
    s_espnow_on = true;
  }
  const uint8_t ch = (uint8_t)WiFi.channel();
  if (ch == 0) {
    return false;
  }
  if (s_peer_ch == ch) {
    return true;
  }
  if (s_peer_ch != 0) {
    esp_now_del_peer(kBridgeMac);
  }
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, kBridgeMac, 6);
  peer.channel = ch;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    return false;
  }
  s_peer_ch = ch;
  Serial.printf("PS4: ESP-NOW peer on WiFi ch %u\n", (unsigned)ch);
  return true;
}

static bool ps4DebouncedEdge(uint32_t now_ms) {
  if ((uint32_t)(now_ms - s_last_edge_ms) < 300) {
    return false;
  }
  s_last_edge_ms = now_ms;
  return true;
}

static float effectiveMaxThrottle() { return s_speed_low ? BOT_PS4_MAX_LIN * 0.5f : BOT_PS4_MAX_LIN; }

static float effectiveMaxSteer() { return s_steer_low ? BOT_PS4_MAX_ANG * 0.5f : BOT_PS4_MAX_ANG; }

static float panTargetFromStick(int8_t rx) {
  const float norm = fleetPs4AxisNorm(rx);
  const float center = static_cast<float>(PAN_CENTER_DEG);
  const float leftSpan = center - static_cast<float>(PAN_MIN_DEG);
  const float rightSpan = static_cast<float>(PAN_MAX_DEG) - center;
  if (norm >= 0.0f) {
    return center + norm * rightSpan;
  }
  return center + norm * leftSpan;
}

static void panStep(uint32_t now_ms, int8_t rx, bool active) {
  if (!active) {
    s_pan_active = false;
    return;
  }
  if (!s_pan_active) {
    s_pan_active = true;
    s_pan_last_ms = 0;
  }
  const float tgt = panTargetFromStick(rx);
  uint32_t dt = 10;
  if (s_pan_last_ms != 0) {
    dt = now_ms - s_pan_last_ms;
    if (dt > 80) {
      dt = 80;
    }
    if (dt < 1) {
      dt = 1;
    }
  }
  s_pan_last_ms = now_ms;
  const float max_step = BOT_PS4_PAN_SLEW_DEG_PER_10MS * (static_cast<float>(dt) / 10.0f);
  float delta = tgt - s_pan_deg;
  if (delta > max_step) {
    delta = max_step;
  } else if (delta < -max_step) {
    delta = -max_step;
  }
  s_pan_deg += delta;
  if (s_pan_deg < static_cast<float>(PAN_MIN_DEG)) {
    s_pan_deg = static_cast<float>(PAN_MIN_DEG);
  }
  if (s_pan_deg > static_cast<float>(PAN_MAX_DEG)) {
    s_pan_deg = static_cast<float>(PAN_MAX_DEG);
  }
}

bool bot_ps4_controls_pan() { return s_pan_active; }

float bot_ps4_pan_deg() { return s_pan_deg; }

void bot_ps4_pan_seed(float deg) {
  if (!s_pan_active) {
    s_pan_deg = deg;
    if (s_pan_deg < static_cast<float>(PAN_MIN_DEG)) {
      s_pan_deg = static_cast<float>(PAN_MIN_DEG);
    }
    if (s_pan_deg > static_cast<float>(PAN_MAX_DEG)) {
      s_pan_deg = static_cast<float>(PAN_MAX_DEG);
    }
  }
}

void bot_ps4_espnow_begin() {
  s_manual = false;
  s_last_pkt_ms = 0;
  s_throttle_now = 0.0f;
  s_speed_low = true;
  s_steer_low = false;
}

bool bot_ps4_manual_active() { return s_manual; }

void bot_ps4_link_status(BotPs4LinkStatus* out) {
  if (!out) {
    return;
  }
  const uint32_t now = millis();
  out->espnow_ready = s_espnow_on && s_peer_ch != 0;
  out->linked = s_last_pkt_ms != 0 && (uint32_t)(now - s_last_pkt_ms) < 2000;
  out->manual = s_manual;
  out->ctrl_ok = s_ctrl_ok;
  out->speed_low = s_speed_low;
  out->steer_low = s_steer_low;
  out->channel = s_peer_ch;
  out->last_pkt_age_ms = s_last_pkt_ms ? (uint32_t)(now - s_last_pkt_ms) : UINT32_MAX;
  out->ack_tx = s_ack_tx;
}

void bot_ps4_espnow_tick(uint32_t now_ms) {
  BotPs4DriveOut out{};

  if (WiFi.status() != WL_CONNECTED) {
    if (s_peer_ch) {
      esp_now_del_peer(kBridgeMac);
      s_peer_ch = 0;
    }
    s_throttle_now = 0.0f;
    s_drive_out = out;
    return;
  }
  if (!ensurePeer()) {
    s_drive_out = out;
    return;
  }

  FleetPs4TxPkt pkt;
  bool got = false;
  portENTER_CRITICAL(&s_rx_mux);
  if (s_rx.fresh) {
    pkt = s_rx.pkt;
    s_rx.fresh = false;
    got = true;
  }
  const uint32_t ack_seq = s_pending_ack_seq;
  s_pending_ack_seq = 0;
  portEXIT_CRITICAL(&s_rx_mux);

  if (ack_seq) {
    sendAck(ack_seq);
  }

  if (got) {
    s_last_pkt_ms = now_ms;
    s_ctrl_ok = (pkt.flags & FLEET_PS4_FLAG_CTRL_OK) != 0;
    s_lx = pkt.lx;
    s_rx_axis = pkt.rx;
    s_l2 = pkt.l2;
    s_r2 = pkt.r2;

    const uint16_t edge = (uint16_t)(pkt.buttons & ~s_buttons_prev);
    s_buttons_prev = pkt.buttons;

    const bool opt_now = (pkt.buttons & FLEET_PS4_BTN_OPTIONS) != 0;
    if (opt_now && !s_opt_prev) {
      s_manual = !s_manual;
      Serial.printf("PS4: manual %s\n", s_manual ? "ON" : "OFF");
    }
    s_opt_prev = opt_now;

    if ((edge & FLEET_PS4_BTN_SQUARE) && ps4DebouncedEdge(now_ms)) {
      s_steer_low = !s_steer_low;
      s_pending_rumble = true;
      Serial.printf("PS4: steer mode %s\n", s_steer_low ? "LOW" : "HIGH");
    }
    if ((edge & FLEET_PS4_BTN_CIRCLE) && ps4DebouncedEdge(now_ms)) {
      s_speed_low = !s_speed_low;
      s_pending_rumble = true;
      Serial.printf("PS4: speed mode %s\n", s_speed_low ? "LOW" : "HIGH");
    }
  }

  const bool ctrl_lost = s_manual && !s_ctrl_ok;
  const bool link_failsafe = fleetPs4LinkFailsafe(now_ms, s_last_pkt_ms, s_manual, s_ctrl_ok);
  const bool pan_active = s_manual && !ctrl_lost && !link_failsafe;
  panStep(now_ms, s_rx_axis, pan_active);

  if (s_manual && !ctrl_lost && !link_failsafe) {
    const auto sp = fleetPs4ScaleManual(s_l2, s_r2, s_lx, effectiveMaxThrottle(), effectiveMaxSteer(),
                                        s_throttle_now);
    s_throttle_now = sp.throttle;
    out.override_ros = true;
    out.linear_x = sp.throttle;
    out.angular_z = sp.steering;
  } else if (s_manual && ctrl_lost) {
    s_throttle_now = 0.0f;
    out.override_ros = true;
    out.linear_x = 0.0f;
    out.angular_z = 0.0f;
  } else if (s_manual && link_failsafe) {
    s_throttle_now = 0.0f;
    out.failsafe = true;
    out.override_ros = false;
  } else {
    s_throttle_now = 0.0f;
  }

  s_drive_out = out;
}

BotPs4DriveOut bot_ps4_espnow_poll(uint32_t now_ms) {
  (void)now_ms;
  return s_drive_out;
}

#endif