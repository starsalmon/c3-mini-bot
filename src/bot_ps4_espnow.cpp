#include "bot_ps4_espnow.h"

#if defined(MICRO_ROS_WIFI) && defined(ENABLE_PS4_ESPNOW)

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include <math.h>
#include <string.h>

#include "fleet_ps4_pkt.h"

#ifndef BOT_PS4_MAX_LIN
#define BOT_PS4_MAX_LIN 0.22f
#endif

#ifndef BOT_PS4_MAX_ANG
#define BOT_PS4_MAX_ANG 0.42f
#endif

#ifndef BOT_PS4_STEER_EXPO
#define BOT_PS4_STEER_EXPO 0.55f
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
static uint32_t s_last_pkt_ms = 0;
static int8_t s_ly = 0;
static int8_t s_lx = 0;
static uint16_t s_buttons_prev = 0;
static bool s_opt_prev = false;
static volatile uint32_t s_pending_ack_seq = 0;
static BotPs4DriveOut s_drive_out;
static uint32_t s_ack_tx = 0;

static bool macEq(const uint8_t* a, const uint8_t* b) { return memcmp(a, b, 6) == 0; }

static void sendAck(uint32_t seq) {
  FleetPs4AckPkt ack{};
  ack.channel = (uint8_t)WiFi.channel();
  ack.seq = seq;
  WiFi.macAddress(ack.mac);
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

static float steerExpo(int8_t lx) {
  const float x = fleetPs4AxisNorm(lx);
  const float expo = BOT_PS4_STEER_EXPO;
  const float y = x * (1.0f - expo) + x * x * x * expo;
  return y * BOT_PS4_MAX_ANG;
}

void bot_ps4_espnow_begin() {
  s_manual = false;
  s_last_pkt_ms = 0;
}

bool bot_ps4_manual_active() { return s_manual; }

void bot_ps4_espnow_tick(uint32_t now_ms) {
  BotPs4DriveOut out{};

  if (WiFi.status() != WL_CONNECTED) {
    if (s_peer_ch) {
      esp_now_del_peer(kBridgeMac);
      s_peer_ch = 0;
    }
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
    s_ly = pkt.ly;
    s_lx = pkt.lx;

    const uint16_t edge = (uint16_t)(pkt.buttons & ~s_buttons_prev);
    s_buttons_prev = pkt.buttons;

    if ((edge & FLEET_PS4_BTN_OPTIONS) != 0) {
      s_manual = !s_manual;
      Serial.printf("PS4: manual %s\n", s_manual ? "ON" : "OFF");
    }
    const bool opt_now = (pkt.buttons & FLEET_PS4_BTN_OPTIONS) != 0;
    s_opt_prev = opt_now;
  }

  const bool link_failsafe =
      fleetPs4LinkFailsafe(now_ms, s_last_pkt_ms, s_manual, s_ctrl_ok);

  if (s_manual && s_ctrl_ok && !link_failsafe) {
    const float fwd = -fleetPs4AxisNorm(s_ly);
    const float lin = fwd * BOT_PS4_MAX_LIN;
    const float ang = steerExpo(s_lx);
    out.override_ros = true;
    out.linear_x = lin;
    out.angular_z = ang;
  } else if (s_manual && link_failsafe) {
    out.failsafe = true;
    out.override_ros = false;
  }

  s_drive_out = out;
}

BotPs4DriveOut bot_ps4_espnow_poll(uint32_t now_ms) {
  (void)now_ms;
  return s_drive_out;
}

#endif