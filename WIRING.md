# c3-mini-bot wiring

Board: [Waveshare ESP32-C3 Zero](https://www.waveshare.com/wiki/ESP32-C3-Zero)

**Source of truth:** `include/bot_pins.h` (GPIO numbers below match firmware defaults).

Always use **BCM / GPIO numbers** in software — not physical header positions.

---

## Quick pin map

| GPIO | Function | Notes |
|------|----------|--------|
| **0** | Wheel L servo signal | FS90R continuous rotation |
| **1** | Wheel R servo signal | FS90R |
| **3** | Pan servo signal | FS90 — sonar + IR mount |
| **2** | Battery sense (ADC) | Divider tap → `/bot1/battery/voltage` |
| **5** | HC-SR04 TRIG | 3.3 V logic |
| **6** | HC-SR04 ECHO | **Divider required** if module outputs 5 V |
| **4** | Passive piezo speaker | `SPEAKER_PIN` — fleet startup / peer ping |
| **7** | TSOP4138 IR receive | Pan head (`IR_RECV_RIGHT_PIN`) |
| **8** | I2C SDA | MPU6050, VL53L0X, SSD1306 OLED |
| **9** | I2C SCL | MPU6050, VL53L0X, SSD1306 OLED |
| **10** | Onboard WS2812 | Status LED (do not repurpose) |
| **19** | Front NeoPixel bar DIN | Optional 8× WS2812 (`FRONT_BAR_PIN`) |
| **20** | IR LED TX | Fleet 38 kHz (`FLEET_IR_ID=1`); USB CDC leaves pin free |
| **21** | USB-UART RX | Console — avoid other use |

Left TSOP channel is disabled in firmware (`IR_RECV_LEFT_PIN = -1`). Set `IR_DUAL=1` in fleet brains only after adding a second fixed TSOP.

---

## Power

| Rail | Connect |
|------|---------|
| **Servos** | Shared **5 V BEC/UBEC** + GND — **not** from C3 3.3 V |
| **C3** | USB or 5 V in per Waveshare guide |
| **MPU6050 / TSOP / VL53L0X** | **3.3 V** + GND |
| **HC-SR04** | **5 V** + GND (ECHO divided to 3.3 V) |

Common **GND** between C3, BEC, and sensors.

---

## Servos (FS90R ×2, FS90 ×1)

| Servo | Signal → C3 | Power |
|-------|-------------|-------|
| Wheel L (FS90R) | **GPIO0** | VBAT + GND |
| Wheel R (FS90R) | **GPIO1** | VBAT + GND |
| Pan (FS90) | **GPIO3** | 5 V BEC + GND |

FS90R: **1500 µs = stop** (`SERVO_STOP_US`). One wheel may need invert in firmware (`SERVO_INVERT_RIGHT=1` default).

---

## HC-SR04 (on pan servo)

| Sensor | C3 |
|--------|-----|
| VCC | 5 V |
| GND | GND |
| TRIG | **GPIO5** |
| ECHO | **GPIO6** |

5 V ECHO → **3.3 V** divider, e.g. **10 kΩ** (ECHO → GPIO6) + **20 kΩ** (GPIO6 → GND).

Mount on the **pan bracket**; firmware publishes `/bot1/sonar/range` and `/bot1/sonar/pan_deg`.

---

## Rear time-of-flight (VL53L0X)

Same I2C bus as the IMU. Leave **XSHUT** floating (addr **0x29**).

| VL53L0X | C3 |
|---------|-----|
| VIN / VCC | **3.3 V** |
| GND | GND |
| SDA | **GPIO8** |
| SCL | **GPIO9** |

Topic: `/bot1/tof/rear`. Reverse stops below ~12 cm (firmware + brain).

If the chip is a **VL53L1X** (model id not `0xEE`), it will log and stay unused until we add that driver.

---

## MPU6050 (I2C)

| MPU6050 | C3 (board labels SDA/SCL) |
|---------|----------------------------|
| SDA | **GPIO8** |
| SCL | **GPIO9** |
| VCC | **3.3 V** |
| GND | GND |

Address **0x68**. Topic: `/bot1/imu`.

PCB layout rotated the chip vs the breadboard. Firmware flips yaw with `IMU_GYRO_Z_SIGN` (`-1`). If heading hold still piles into a turn, set it to `1` in `bot_pins.h`.

---

## IR receiver — TSOP4138 on pan

Lens toward you, legs down: **1 = OUT, 2 = GND, 3 = VCC**.

| TSOP leg | Connect |
|----------|---------|
| **OUT** | **GPIO7** |
| **GND** | GND |
| **VCC** | **3.3 V** |

While driving, firmware does occasional sonar pan glances; on IR hit it holds bearing (`IR_PAN_SCAN=1`). Topics: `/bot1/ir/detected`, `/bot1/ir/peer` (fleet ID of other bots).

---

## IR LED beacon (optional TX)

For swarm chase — rover beacon is separate hardware; mini can TX when driving (`IR_TX_FOLLOW_CMD=1`).

| C3 | Circuit |
|----|---------|
| **GPIO20** | **100–150 Ω** → IR LED (+) → IR LED (−) → GND |

For more range: GPIO → **1 kΩ** → NPN base; IR LED + resistor from **5 V** → collector; emitter → GND.

38 kHz PWM (`IR_TX_HZ`). Beacon on/off is a **PlatformIO flag** in `platformio_private.ini`:

- `-DIR_TX_DEFAULT_ON=1` — on at boot (chase target)
- `-DIR_TX_FOLLOW_CMD=1` — only while driving

Default in `bot_pins.h` is off (`0`) so rover front IR does not treat the mini as a bumper.

---

## Battery monitor

| Node | Connect |
|------|---------|
| Cell **+** | R1 (e.g. **27 kΩ**) → **GPIO2** → R2 (e.g. **10 kΩ**) → GND |

Report-only on ROS — no auto shutdown (`BAT_SHUTDOWN_ENABLE=0`). Firmware auto-picks a 1S scale from the tap millivolts (`BAT_ADC_SCALE=0`). DMM-check once on the PCB and set `BAT_ADC_SCALE` in `bot_pins.h` if the reading is a few tenths off.

---

## SSD1306 OLED (128×64, I2C)

Same bus as MPU and rear ToF. Address **0x3C** (`OLED_I2C_ADDR`).

| OLED module | C3 |
|-------------|-----|
| VCC | **3.3 V** |
| GND | GND |
| SDA | **GPIO8** |
| SCL | **GPIO9** |

Firmware splash in `oled_ssd1306.cpp` when `OLED_ENABLED=1` (default in `c3_mini_wifi*` envs).

---

## Speaker (passive piezo)

| C3 | Connect |
|----|---------|
| **GPIO4** | Buzzer + |
| GND | Buzzer − |

Startup chirp and short ping when another fleet IR ID is decoded (`bot_speaker.cpp`). Avoid strong loads on GPIO4 at reset (C3 strap).

---

## Status LED (GPIO10)

Onboard WS2812 — boot / WiFi / agent / IR lock colours in `bot_status_led.cpp`. No extra wiring.

---

## micro-ROS (off-board brain)

```
┌─────────────┐   WiFi UDP :8888   ┌──────────────────┐
│  C3 Zero    │ ◄────────────────► │  dockerhost      │
│  bot1/*     │                    │  microros-agent  │
└─────────────┘                    │  explore/chase   │
                                   └──────────────────┘
```

Fleet env: `BOT_NAMESPACE=bot1`, `AGENT_IP=192.168.3.172`, `FLEET_IR_ID=1` in `platformio.ini` / `platformio_private.ini`. See `docs/SWARM.md` and `docs/PINMAP.md`.

USB serial transport works for bench without WiFi.

---

## Bench / flash

```bash
pio run -e c3_mini_wifi_ota_usb -t upload    # first USB flash
pio run -e c3_mini_wifi_ota -t upload        # OTA (hostname c3-bot1.local)
pio run -e c3_bench -t upload                  # battery + servo bench
```

---

## Related

- `docs/PINMAP.md` — build-oriented pin map (PDF-ready)
- `docs/SWARM.md` — fleet + rover IR coordination
- `include/bot_pins.h` — all `#define` overrides
