#include "bot_servo_web.h"

#if defined(ENABLE_SERVO_WEB) && defined(MICRO_ROS_WIFI)

#include <WebServer.h>
#include <WiFi.h>

#include <stdio.h>

#include "bot_fw_version.h"
#include "bot_ota.h"
#include "bot_pins.h"
#include "drive_cal_store.h"
#include "drive_servos.h"

namespace {

WebServer server(80);
DriveServos* s_drive = nullptr;
bool s_routes_registered = false;
bool s_listening = false;

const char kPage[] PROGMEM = R"html(<!DOCTYPE html>
<html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Roby wheel neutral</title>
<style>
body{font-family:sans-serif;margin:12px;background:#111;color:#eee}
h1{font-size:1.2rem}
.row{margin:10px 0;padding:10px;background:#222;border-radius:8px}
button{min-width:48px;min-height:44px;margin:4px;font-size:1rem}
.big{display:block;width:100%;margin:8px 0;padding:14px}
.val{font-size:1.4rem;font-weight:bold}
</style></head><body>
<h1>FS90R neutral trim</h1>
<p id="fw"></p>
<p>Hold stop, nudge &micro;s until no creep, then Save. cmd_vel cancels hold.</p>
<div class="row"><b>Left</b> <span class="val" id="l">—</span> &micro;s
<br>
<button onclick="n('L',-5)">-5</button><button onclick="n('L',-1)">-1</button>
<button onclick="n('L',1)">+1</button><button onclick="n('L',5)">+5</button></div>
<div class="row"><b>Right</b> <span class="val" id="r">—</span> &micro;s
<br>
<button onclick="n('R',-5)">-5</button><button onclick="n('R',-1)">-1</button>
<button onclick="n('R',1)">+1</button><button onclick="n('R',5)">+5</button></div>
<div class="row">Slew/10ms: <span id="slew">—</span>
<button onclick="adj('slew',-1)">-</button><button onclick="adj('slew',1)">+</button>
<br>Max offset &micro;s: <span id="off">—</span>
<button onclick="adj('off',-10)">-10</button><button onclick="adj('off',10)">+10</button></div>
<button class="big" onclick="post('/api/hold')">Hold stop (8s)</button>
<button class="big" onclick="post('/api/save')">Save to NVS</button>
<p id="st"></p>
<script>
async function refresh(){const j=await (await fetch('/api/state')).json();
document.getElementById('fw').textContent='fw '+j.fw+' built '+j.built;
document.getElementById('l').textContent=j.l;document.getElementById('r').textContent=j.r;
document.getElementById('slew').textContent=j.slew;document.getElementById('off').textContent=j.off;
document.getElementById('st').textContent=(j.hold?'Hold active':'Ready')+' OTA='+(j.ota_ok?'up':'down');
async function n(w,d){await fetch('/api/nudge?w='+w+'&d='+d,{method:'POST'});refresh();}
async function adj(k,d){await fetch('/api/adj?k='+k+'&d='+d,{method:'POST'});refresh();}
async function post(p){await fetch(p,{method:'POST'});refresh();}
refresh();setInterval(refresh,800);
</script></body></html>)html";

void send_state() {
  if (!s_drive) {
    server.send(500, "application/json", "{}");
    return;
  }
  const BotOtaStatus ota = bot_ota_status();
  const DriveCalStore::BootCalStatus& boot = DriveCalStore::boot_cal_status();
  char buf[320];
  snprintf(buf, sizeof(buf),
           "{\"l\":%u,\"r\":%u,\"slew\":%u,\"off\":%u,\"hold\":%s,"
           "\"fw\":\"%s\",\"built\":\"%s\",\"ota_ok\":%s,\"ota_busy\":%s,"
           "\"cal_boot\":%s,\"cal_magic\":%lu}",
           s_drive->stop_us_left(), s_drive->stop_us_right(), s_drive->slew_us_per_10ms(),
           s_drive->max_offset_us(), s_drive->hold_stop_active() ? "true" : "false", BOT_FW_GIT_HASH,
           BOT_FW_BUILD_TIME, ota.ota_listening ? "true" : "false", ota.ota_busy ? "true" : "false",
           boot.load_ok ? "true" : "false", (unsigned long)boot.magic_read);
  server.send(200, "application/json", buf);
}

void handle_nvs() {
  DriveCalStore::NvsSnapshot nvs{};
  const bool snap = DriveCalStore::read_nvs_snapshot(&nvs);
  const DriveCalStore::BootCalStatus& boot = DriveCalStore::boot_cal_status();
  char buf[280];
  snprintf(buf, sizeof(buf),
           "{\"nvs_open\":%s,\"magic\":%lu,\"stop_l\":%u,\"stop_r\":%u,\"slew\":%u,\"maxoff\":%u,"
           "\"boot_load_ok\":%s,\"boot_magic\":%lu,\"servos_attached\":%s}",
           snap && nvs.open_ok ? "true" : "false", (unsigned long)nvs.magic, nvs.stop_l, nvs.stop_r,
           nvs.slew, nvs.maxoff, boot.load_ok ? "true" : "false", (unsigned long)boot.magic_read,
           boot.servos_attached ? "true" : "false");
  server.send(200, "application/json", buf);
}

void handle_root() {
  server.send_P(200, "text/html", kPage);
}

void handle_nudge() {
  if (!s_drive) {
    server.send(500, "text/plain", "no drive");
    return;
  }
  const String w = server.arg("w");
  const int d = server.arg("d").toInt();
  uint16_t l = s_drive->stop_us_left();
  uint16_t r = s_drive->stop_us_right();
  if (w == "L" || w == "l") {
    l = constrain(static_cast<int>(l) + d, SERVO_MIN_US, SERVO_MAX_US);
  } else {
    r = constrain(static_cast<int>(r) + d, SERVO_MIN_US, SERVO_MAX_US);
  }
  s_drive->set_stop_us(l, r);
  if (s_drive->hold_stop_active()) {
    s_drive->begin_hold_stop(SERVO_HOLD_STOP_MS);
  }
  server.send(200, "text/plain", "ok");
}

void handle_adj() {
  if (!s_drive) {
    server.send(500, "text/plain", "no drive");
    return;
  }
  const String k = server.arg("k");
  const int d = server.arg("d").toInt();
  uint16_t slew = s_drive->slew_us_per_10ms();
  uint16_t off = s_drive->max_offset_us();
  if (k == "slew") {
    slew = constrain(static_cast<int>(slew) + d, 1, 40);
  } else {
    off = constrain(static_cast<int>(off) + d, 50, 500);
  }
  s_drive->set_motion_limits(slew, off);
  server.send(200, "text/plain", "ok");
}

void handle_hold() {
  if (!s_drive) {
    server.send(500, "text/plain", "no drive");
    return;
  }
  s_drive->begin_hold_stop(SERVO_HOLD_STOP_MS);
  server.send(200, "text/plain", "ok");
}

void handle_save() {
  if (!s_drive) {
    server.send(500, "text/plain", "no drive");
    return;
  }
  const DriveCalStore::Settings s = DriveCalStore::from_drive(*s_drive);
  if (DriveCalStore::save(s)) {
    server.send(200, "application/json",
                 "{\"ok\":true,\"l\":" + String(s.stop_us_left) + ",\"r\":" +
                     String(s.stop_us_right) + "}");
  } else {
    server.send(500, "application/json", "{\"ok\":false}");
  }
}

void register_routes() {
  if (s_routes_registered) {
    return;
  }
  server.on("/", HTTP_GET, handle_root);
  server.on("/api/state", HTTP_GET, send_state);
  server.on("/api/nudge", HTTP_POST, handle_nudge);
  server.on("/api/adj", HTTP_POST, handle_adj);
  server.on("/api/hold", HTTP_POST, handle_hold);
  server.on("/api/save", HTTP_POST, handle_save);
  server.on("/api/nvs", HTTP_GET, handle_nvs);
  s_routes_registered = true;
}

void stop_listening() {
  if (s_listening) {
    server.close();
    s_listening = false;
  }
}

void start_listening() {
  if (WiFi.status() != WL_CONNECTED) {
    stop_listening();
    return;
  }
  WiFi.setSleep(WIFI_PS_NONE);
  register_routes();
  if (s_listening) {
    server.close();
    s_listening = false;
  }
  server.begin();
  s_listening = true;
  Serial.printf("Servo cal web http://%s/\n", WiFi.localIP().toString().c_str());
}

}  // namespace

void bot_servo_web_begin(DriveServos& drive) {
  s_drive = &drive;
  register_routes();
  start_listening();
}

void bot_servo_web_tick() {
  if (WiFi.status() != WL_CONNECTED) {
    stop_listening();
    return;
  }
  if (!s_listening) {
    start_listening();
  }
  if (s_listening) {
    server.handleClient();
  }
}

void bot_servo_web_on_wifi_event(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      stop_listening();
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      start_listening();
      break;
    default:
      break;
  }
}

void bot_servo_web_session_reset() {
  if (WiFi.status() == WL_CONNECTED) {
    start_listening();
  }
}

#endif