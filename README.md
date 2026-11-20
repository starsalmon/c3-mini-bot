# c3-mini-bot (Roby)

ESP32-C3 Zero **thin client** for the fleet brain on **dockerhost** (micro-ROS agent UDP **8888**, ROS 2 Jazzy, domain 0). Roby is **fleet IR ID 1**, ROS namespace **`bot1`**.

**Pin and wiring source of truth:** `include/bot_pins.h` and `docs/PINMAP.md`. Human-readable wiring: `WIRING.md`. Future ideas: [docs/TODO.md](docs/TODO.md).

## Hardware (fleet build)

| Part | Role |
|------|------|
| Waveshare ESP32-C3 Zero | WiFi micro-ROS, OTA |
| 2× FS90R | Wheel servos |
| FS90 | Pan (sonar + TSOP) |
| HC-SR04 | Pan-mounted ultrasonic |
| MPU6050 | IMU on I2C |
| VL53L0X | Rear ToF on I2C |
| SSD1306 128×64 | OLED on I2C 0x3C |
| TSOP4138 | IR receive (fleet + beacon) |
| IR LED | Fleet IR TX (38 kHz, ID 1) |
| Passive piezo | Speaker on GPIO4 |
| Onboard WS2812 (GPIO10) | Fleet WiFi / micro-ROS status (amber connecting, blue no agent, green OK) |
| Optional 8× NeoPixel bar | Motion hints on GPIO19 (open hardware issue — not used for fleet status) |

## Quick start

### Fleet firmware (normal operation)

```bash
cp platformio_private.ini.example platformio_private.ini
# Set WIFI_*, AGENT_IP=192.168.3.172, BOT_NAMESPACE=bot1

pio run -e c3_mini_wifi_ota_usb -t upload   # first USB flash only (USB CDC on for flash + serial)
pio run -e c3_mini_wifi_ota -t upload       # OTA — stop motion first (see OTA safety)
```

Fleet firmware uses the **`min_spiffs`** partition table (~1.9 MB app). **Changing partition layout requires a one-time USB flash** (`c3_mini_wifi_ota_usb`); OTA cannot resize the partition table in place.

**`c3_mini_wifi` / `c3_mini_wifi_ota`:** USB CDC is **off** so **GPIO19** drives the front NeoPixel bar; use OTA or UART **GPIO20/21** for logs. **`c3_mini_wifi_ota_usb`**, **`c3_bench`**, and **`c3_mini_serial`** keep USB CDC for esptool / bench / micro-ROS serial (front bar on GPIO19 may not work on those builds while CDC owns D+).

On a fresh PlatformIO install, pioarduino sometimes fetches `framework-arduinoespressif32-libs` before the matching `framework-arduinoespressif32` tarball. If the first `pio run` errors on missing Arduino core headers, run `pio pkg install -e c3_mini_wifi_ota` once, then build again.

On dockerhost: `docker compose up -d` (agent + fleet-brain). See `docs/SWARM.md`.

### Bench / bring-up

| Env | Use |
|-----|-----|
| `c3_bench` | Serial keys: wheels, pan, sonar, I2C scan |
| `c3_autonomous` | Standalone wander on ESP (no WiFi / no brain) |

### USB serial micro-ROS (no WiFi)

```bash
pio run -e c3_mini_serial -t upload
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

## ROS topics (namespace `bot1`)

Topics are relative in firmware; the agent exposes them under `/bot1/…`:

| Topic | Type | Direction |
|-------|------|-----------|
| `cmd_vel` | `geometry_msgs/Twist` | → ESP |
| `sonar/range` | `sensor_msgs/Range` | ← ESP |
| `sonar/pan_deg` | `std_msgs/Float32` | ← ESP |
| `imu` | `sensor_msgs/Imu` | ← ESP |
| `ir/detected` | `std_msgs/Bool` | ← ESP |
| `ir/peer` | `std_msgs/UInt8` | ← fleet peer ID |
| `battery/voltage` | `std_msgs/Float32` | ← ESP |
| `tof/rear` | `sensor_msgs/Range` | ← ESP |
| `stall` | `std_msgs/Bool` | ← ESP |

### Fleet QoS (micro-ROS on ESP)

High-rate streams use **best effort** so the XRCE agent is not flooded with reliable ACKs; one-off or edge events stay **reliable**. All listed types are under the WiFi **1400 B** MTU.

| Topic | Reliability |
|-------|-------------|
| `cmd_vel` (sub) | best effort |
| `sonar/range`, `sonar/pan_deg`, `imu`, `ir/detected`, `ir/peer`, `battery/voltage`, `tof/rear` | best effort |
| `stall` | reliable (on-change) |

Brain and tools should match: `ros2/docker/rover-record-qos.yaml`. Firmware uses `rclc_publisher_init_best_effort` / `rclc_subscription_init_best_effort` except `rclc_publisher_init_default` on `stall`.

Tune wheels in `include/bot_pins.h` (`SERVO_STOP_US_*`, trims/inverts). **Neutral trim from phone:** with `c3_mini_wifi_ota` on Wi‑Fi, open **[http://c3-bot1.local/](http://c3-bot1.local/)** or **[http://192.168.3.181/](http://192.168.3.181/)** → Hold stop → nudge µs → Save (NVS).

### PS4 manual drive (ESP-NOW)

Same **NodeMCU-32S bridge** as Jazz (`starsalmon/jazz`, branch `cursor/fleet-rgb-status-498d`). Flash the fleet bridge env so you can pick Roby vs Jazz:

```bash
cd jazz/bridge
pio run -e ps4_bridge_fleet -t upload
```

Roby firmware (`c3_mini_wifi_ota`) includes the ESP-NOW receiver on the **home Wi‑Fi channel** (`WiFi.setSleep(WIFI_PS_NONE)`; peer locked to bridge MAC `e0:8c:fe:f9:c2:34`).

| Control | Roby (same mapping as Jazz ProS3) |
|---------|------|
| **Options** | Toggle **manual** ON/OFF (when ON, overrides `/bot1/cmd_vel`) |
| **R2** | Forward throttle (analog) |
| **L2** | Brake when moving forward; reverse once near zero (net R2 − L2) |
| **Left stick X** | Steer (expo ~0.6 after deadzone) |
| **Right stick X** | Sonar pan (position across 30°–150°, centred at 90°; stick centre = pan centre) |
| **Circle** | Toggle speed **LOW/HIGH** (LOW = 50% max forward; default **LOW**) |
| **Square** | Toggle steer **LOW/HIGH** (LOW = 50% turn; default **HIGH**) |
| **Share** (on bridge) | Cycle ESP-NOW target Jazz ↔ Roby (`ps4_bridge_fleet` only) |
| **Triangle / Cross / L1 / R1** | Jazz-only features (balance, pivot, horns) — **no effect** on Roby |
| Link loss **300 ms** | Wheels stop; ROS `cmd_vel` resumes |

**Heading hold (MPU6050 on I2C GPIO8/9):** When PS4 manual or ROS `cmd_vel` is driving straight (steer in deadzone, throttle above ~6%), firmware latches yaw and adds a PI+PD correction on wheel differential. Gyro bias is learned at boot while still (~2.5 s). Tune **Kp/Ki/Kd** and enable on the cal web page; NVS namespace `hdg_hold`. Reverse driving flips correction sign.

Ack feedback on the pad (via bridge): light bar **blue** = speed LOW, **green** = speed HIGH; short rumble on Circle/Square.

Roby mount may need opposite stick X sense vs Jazz; defaults in `bot_pins.h` (`BOT_PS4_STEER_SIGN`, `BOT_PS4_PAN_SIGN`, both **-1.0**). Override in `platformio_private.ini`, e.g. `-DBOT_PS4_STEER_SIGN=1.0f`.

Stop the bot before OTA. After OTA reboot, servo cal HTTP should come up automatically (Wi‑Fi events register before connect).

**Firmware identity (no USB serial):** cal page shows firmware hash and build time; `GET /api/nvs` for raw NVS. Tarball builds without `.git`: `FW_GIT_HASH=abc123 pio run -e c3_mini_wifi_ota` or put the hash in a one-line `FW_VERSION` file in the repo root.

**USB flash when OTA is stuck:** `pio run -e c3_mini_wifi_ota_usb -t upload` (hold BOOT if needed).
