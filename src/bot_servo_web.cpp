#include "bot_servo_web.h"

#if defined(ENABLE_SERVO_WEB) && defined(MICRO_ROS_WIFI)

#include <WebServer.h>
#include <WiFi.h>

#include <stdio.h>

#include "bot_fw_version.h"
#include "bot_ota.h"
#include "bot_pins.h"
#include "bot_ps4_espnow.h"
#include "drive_cal_store.h"
#include "drive_servos.h"

namespace {

WebServer server(80);
DriveServos* s_drive = nullptr;
bool s_routes_registered = false;
bool s_listening = false;
float s_pan_deg = 90.0f;

const char kPage[] PROGMEM = R"html(<!DOCTYPE html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Roby wheel calibration</title>
<style>
:root{--bg:#0f1115;--card:#1a1f28;--muted:#9aa4b2;--ok:#3dd68c;--warn:#f5a524;--text:#eef1f5}
*{box-sizing:border-box}
body{font-family:system-ui,-apple-system,sans-serif;margin:0;padding:12px;max-width:480px;margin-inline:auto;
  background:var(--bg);color:var(--text);line-height:1.45;font-size:16px}
h1{font-size:1.25rem;margin:0 0 8px}
.sub{color:var(--muted);font-size:.9rem;margin-bottom:12px}
.card{background:var(--card);border-radius:12px;padding:14px;margin:10px 0}
.card h2{font-size:1rem;margin:0 0 10px;color:#c8d0dc}
.row{display:flex;justify-content:space-between;align-items:center;gap:8px;margin:8px 0;flex-wrap:wrap}
.val{font-size:1.35rem;font-weight:600;white-space:nowrap}
.lbl{color:var(--muted);font-size:.85rem;flex:1;min-width:140px}
.btns{display:flex;flex-wrap:wrap;gap:6px;margin-top:8px}
button{min-width:52px;min-height:48px;padding:10px 14px;font-size:1rem;border-radius:10px;border:1px solid #3a4556;
  background:#252b36;color:var(--text);touch-action:manipulation}
button:active{background:#323a48}
.big{display:block;width:100%;margin:8px 0;padding:16px;font-weight:600}
.status{padding:10px;border-radius:10px;background:#12151c;font-size:.9rem}
.ok{color:var(--ok)}.warn{color:var(--warn)}
details{margin-top:10px;font-size:.85rem;color:var(--muted)}
details summary{cursor:pointer;padding:6px 0}
#saveBtn{background:#1e3a2f;border-color:#3dd68c}
</style></head><body>
<h1>Wheel neutral calibration</h1>
<p class="sub" id="hint">While wheels are held still, nudge pulse width until each wheel stops creeping. Save stores values in flash (NVS).</p>
<div class="status" id="banner"></div>
<div class="card">
  <h2>Left wheel — stop pulse</h2>
  <div class="row"><span class="lbl">Pulse width (center / no motion)</span><span class="val" id="l">—</span></div>
  <div class="btns">
    <button type="button" onclick="n('L',-5)">−5 µs</button>
    <button type="button" onclick="n('L',-1)">−1</button>
    <button type="button" onclick="n('L',1)">+1</button>
    <button type="button" onclick="n('L',5)">+5</button>
  </div>
</div>
<div class="card">
  <h2>Right wheel — stop pulse</h2>
  <div class="row"><span class="lbl">Pulse width (center / no motion)</span><span class="val" id="r">—</span></div>
  <div class="btns">
    <button type="button" onclick="n('R',-5)">−5 µs</button>
    <button type="button" onclick="n('R',-1)">−1</button>
    <button type="button" onclick="n('R',1)">+1</button>
    <button type="button" onclick="n('R',5)">+5</button>
  </div>
</div>
<div class="card">
  <h2>Motion smoothing</h2>
  <div class="row"><span class="lbl">Ramp speed</span><span class="val" id="slew">—</span></div>
  <div class="btns"><button type="button" onclick="adj('slew',-1)">Slower</button><button type="button" onclick="adj('slew',1)">Faster</button></div>
  <div class="row" style="margin-top:12px"><span class="lbl">Max speed offset from stop</span><span class="val" id="off">—</span></div>
  <div class="btns"><button type="button" onclick="adj('off',-10)">−10 µs</button><button type="button" onclick="adj('off',10)">+10 µs</button></div>
</div>
<button class="big" type="button" onclick="post('/api/hold')">Hold wheels stopped (8 s)</button>
<button class="big" id="saveBtn" type="button" onclick="save()">Save to flash (NVS)</button>
<div class="card">
  <h2>Wireless update &amp; PS4</h2>
  <div id="sys"></div>
  <div class="row" style="margin-top:10px"><span class="lbl">Sonar pan angle</span><span class="val" id="pan">—</span></div>
</div>
<details><summary>Technical details (NVS, build)</summary>
  <pre id="detail" style="white-space:pre-wrap;font-size:.8rem"></pre>
</details>
<script>
function fmtSlew(v){return v+' µs per 10 ms';}
function fmtOff(v){return '±'+v+' µs';}
function pktAge(ms){if(ms>60000)return 'never';return ms<1000?ms+' ms ago':Math.round(ms/1000)+' s ago';}
async function refresh(){
  const j=await (await fetch('/api/state')).json();
  document.getElementById('l').textContent=j.l+' µs';
  document.getElementById('r').textContent=j.r+' µs';
  document.getElementById('slew').textContent=fmtSlew(j.slew);
  document.getElementById('off').textContent=fmtOff(j.off);
  const ban=document.getElementById('banner');
  if(j.hold){ban.innerHTML='<span class="warn">Holding stop — nudge now. cmd_vel cancels hold.</span>';}
  else if(j.cal_saved){ban.innerHTML='<span class="ok">Saved in flash — matches NVS</span>';}
  else{ban.innerHTML='<span class="warn">Unsaved changes — tap Save when trimming looks good</span>';}
  const ota=j.ota_ok?'<span class="ok">OTA ready</span>':'<span class="warn">OTA not listening</span>';
  let ps4='PS4 bridge: ';
  if(!j.ps4_ready){ps4+='radio not ready';}
  else if(!j.ps4_link){ps4+='no recent packets';}
  else{ps4+=(j.ps4_manual?'MANUAL on, ':'manual off, ')+'pkt '+pktAge(j.ps4_pkt_age_ms);}
  if(j.ps4_speed_low)ps4+=', speed LOW';
  if(j.ps4_steer_low)ps4+=', steer LOW';
  document.getElementById('pan').textContent=(j.pan_deg!=null)?(Number(j.pan_deg).toFixed(1)+'°'):'—';
  document.getElementById('sys').innerHTML=
    '<div>Firmware <b>'+j.fw+'</b> · built '+j.built+'</div>'+
    '<div>'+ota+(j.ota_busy?', <span class="warn">OTA busy</span>':'')+'</div>'+
    '<div>'+ps4+'</div>';
  document.getElementById('detail').textContent=
    'NVS stop L/R: '+j.nvs_l+' / '+j.nvs_r+' µs · magic '+(j.nvs_valid?'0x'+j.nvs_magic_hex.toString(16).toUpperCase():'invalid')+
    '\\nBoot load ok: '+j.cal_boot;
}
async function n(w,d){await fetch('/api/nudge?w='+w+'&d='+d,{method:'POST'});refresh();}
async function adj(k,d){await fetch('/api/adj?k='+k+'&d='+d,{method:'POST'});refresh();}
async function post(p){await fetch(p,{method:'POST'});refresh();}
async function save(){await fetch('/api/save',{method:'POST'});refresh();}
refresh();setInterval(refresh,900);
</script></body></html>)html";

static bool cal_matches_nvs(const DriveServos& d, const DriveCalStore::NvsSnapshot& nvs) {
  if (nvs.magic != DriveCalStore::kMagic) {
    return false;
  }
  return d.stop_us_left() == nvs.stop_l && d.stop_us_right() == nvs.stop_r &&
         d.slew_us_per_10ms() == nvs.slew && d.max_offset_us() == nvs.maxoff;
}

void send_state() {
  if (!s_drive) {
    server.send(500, "application/json", "{}");
    return;
  }
  const BotOtaStatus ota = bot_ota_status();
  const DriveCalStore::BootCalStatus& boot = DriveCalStore::boot_cal_status();
  DriveCalStore::NvsSnapshot nvs{};
  DriveCalStore::read_nvs_snapshot(&nvs);
  const bool nvs_valid = nvs.open_ok && nvs.magic == DriveCalStore::kMagic;
  const bool cal_saved = cal_matches_nvs(*s_drive, nvs);

  BotPs4LinkStatus ps4{};
  bot_ps4_link_status(&ps4);
  uint32_t pkt_age = ps4.last_pkt_age_ms;
  if (pkt_age == UINT32_MAX) {
    pkt_age = 999999;
  }

  char buf[640];
  snprintf(buf, sizeof(buf),
           "{\"l\":%u,\"r\":%u,\"slew\":%u,\"off\":%u,\"hold\":%s,"
           "\"fw\":\"%s\",\"built\":\"%s\",\"ota_ok\":%s,\"ota_busy\":%s,"
           "\"cal_saved\":%s,\"cal_boot\":%s,"
           "\"nvs_l\":%u,\"nvs_r\":%u,\"nvs_slew\":%u,\"nvs_off\":%u,\"nvs_valid\":%s,"
           "\"nvs_magic_hex\":%lu,"
           "\"ps4_ready\":%s,\"ps4_link\":%s,\"ps4_manual\":%s,\"ps4_ctrl\":%s,"
           "\"ps4_pkt_age_ms\":%lu,\"ps4_speed_low\":%s,\"ps4_steer_low\":%s,\"ps4_ch\":%u,"
           "\"pan_deg\":%.1f}",
           s_drive->stop_us_left(), s_drive->stop_us_right(), s_drive->slew_us_per_10ms(),
           s_drive->max_offset_us(), s_drive->hold_stop_active() ? "true" : "false", BOT_FW_GIT_HASH,
           BOT_FW_BUILD_TIME, ota.ota_listening ? "true" : "false", ota.ota_busy ? "true" : "false",
           cal_saved ? "true" : "false", boot.load_ok ? "true" : "false", nvs.stop_l, nvs.stop_r,
           nvs.slew, nvs.maxoff, nvs_valid ? "true" : "false", (unsigned long)nvs.magic,
           ps4.espnow_ready ? "true" : "false", ps4.linked ? "true" : "false",
           ps4.manual ? "true" : "false", ps4.ctrl_ok ? "true" : "false", (unsigned long)pkt_age,
           ps4.speed_low ? "true" : "false", ps4.steer_low ? "true" : "false", (unsigned)ps4.channel,
           static_cast<double>(s_pan_deg));
  server.send(200, "application/json", buf);
}

void handle_nvs() {
  DriveCalStore::NvsSnapshot nvs{};
  const bool snap = DriveCalStore::read_nvs_snapshot(&nvs);
  const DriveCalStore::BootCalStatus& boot = DriveCalStore::boot_cal_status();
  char buf[280];
  snprintf(buf, sizeof(buf),
           "{\"nvs_open\":%s,\"magic\":%lu,\"magic_hex\":\"0x%08lX\",\"stop_l\":%u,\"stop_r\":%u,"
           "\"slew\":%u,\"maxoff\":%u,\"boot_load_ok\":%s,\"servos_attached\":%s}",
           snap && nvs.open_ok ? "true" : "false", (unsigned long)nvs.magic, (unsigned long)nvs.magic,
           nvs.stop_l, nvs.stop_r, nvs.slew, nvs.maxoff, boot.load_ok ? "true" : "false",
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

void bot_servo_web_set_pan_deg(float deg) { s_pan_deg = deg; }

#endif