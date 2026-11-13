# Roby pin map

Waveshare ESP32-C3 Zero — fleet mini **Roby** (`BOT_NAMESPACE=bot1`, fleet IR ID **1**, OTA hostname `c3-bot1`, typical LAN IP **192.168.3.181**).

## Boards

| Board | Role | Hostname / IP |
|-------|------|----------------|
| Waveshare ESP32-C3 Zero | Motors, sensors, micro-ROS client, IR, speaker, OLED | `c3-bot1` / `c3-bot1.local` (e.g. 192.168.3.181) |
| dockerhost (NUC) | micro-ROS agent UDP 8888, fleet brain | 192.168.3.172 |

## Waveshare ESP32-C3 Zero

Use **GPIO (BCM)** numbers in firmware (`include/bot_pins.h`).

| Board pin (silk label) | GPIO | Function | Connects to (device + device pin) | Notes |
|------------------------|------|----------|-----------------------------------|-------|
| — | 0 | PWM | FS90R wheel L — signal | Continuous rotation; `SERVO_INVERT_LEFT` |
| — | 1 | PWM | FS90R wheel R — signal | `SERVO_INVERT_RIGHT=1` default |
| A2 / ADC | 2 | ADC | Battery divider tap | 1S LiPo via R1/R2; `/bot1/battery/voltage` |
| — | 3 | PWM | FS90 pan — signal | HC-SR04 + TSOP mount |
| (strap/JTAG) | 4 | LEDC | Passive buzzer + | Piezo; avoid external loads at reset |
| — | 5 | DIG OUT | HC-SR04 TRIG | 3.3 V logic |
| — | 6 | DIG IN | HC-SR04 ECHO | 5 V → divider → ~3.3 V |
| — | 7 | DIG IN | TSOP4138 OUT (pin 1) | LOW = 38 kHz; `IR_RECV_RIGHT_PIN` |
| SDA | 8 | I2C SDA | MPU6050 SDA, VL53L0X SDA, SSD1306 SDA | Shared bus |
| SCL | 9 | I2C SCL | MPU6050 SCL, VL53L0X SCL, SSD1306 SCL | Shared bus |
| LED | 10 | NeoPixel | Onboard WS2812 | Status colours; do not repurpose |
| U0TXD | 20 | LEDC 38 kHz | IR LED anode via 100–150 Ω | Fleet TX; USB CDC leaves this usable for IR |
| U0RXD | 21 | — | USB serial console | Leave unused for GPIO |
| — | 19 | NeoPixel data | Front 8× WS2812 bar DIN | Optional add-on `FRONT_BAR_PIN` |

Disabled / unused in default firmware: `IR_RECV_LEFT_PIN=-1`, `BAT_BUCK_EN_PIN=-1`, `OLED_RESET_PIN=-1` (I2C reset tied high on module).

## I2C devices

| Bus | Address | Device |
|-----|---------|--------|
| GPIO8 / GPIO9 | 0x68 | MPU6050 IMU → `/bot1/imu` |
| GPIO8 / GPIO9 | 0x29 | VL53L0X rear ToF → `/bot1/tof/rear` |
| GPIO8 / GPIO9 | 0x3C | SSD1306 128×64 OLED (`OLED_I2C_ADDR`) |

## Build steps

1. **Power:** Common GND between C3, 5 V BEC (servos + HC-SR04), and 3.3 V sensor rail. C3 from USB or 5 V per Waveshare guide.
2. **Servos:** FS90R ×2 and FS90 pan — signal to GPIO0/1/3; power from shared **5 V** BEC (not C3 3.3 V).
3. **HC-SR04:** 5 V + GND; TRIG → GPIO5; ECHO through divider (e.g. 10 kΩ to GPIO6, 20 kΩ GPIO6 to GND).
4. **I2C bus:** SDA GPIO8, SCL GPIO9, 3.3 V + GND to MPU6050, VL53L0X, SSD1306. Use short leads; many modules include pull-ups — only one 4.7 kΩ pair per bus if needed.
5. **MPU6050:** Addr 0x68; orient per PCB (`IMU_GYRO_Z_SIGN` in firmware).
6. **VL53L0X (rear):** Addr 0x29; leave XSHUT floating unless you add a second ToF later.
7. **SSD1306 OLED:** VCC 3.3 V, GND, SDA/SCL on GPIO8/9; I2C **0x3C**.
8. **TSOP4138 (pan):** Leg 1 OUT → GPIO7, leg 2 GND, leg 3 VCC → 3.3 V; lens forward on pan bracket.
9. **IR LED TX (fleet ID 1):** GPIO20 → 100–150 Ω → LED anode; cathode → GND. For more current: NPN sink from 5 V (see `WIRING.md`).
10. **Speaker:** GPIO4 → passive piezo +; other leg GND.
11. **Battery sense:** Cell + → R1 → GPIO2 → R2 → GND; calibrate `BAT_ADC_SCALE` after DMM check.
12. **Optional front bar:** DIN → GPIO19; 5 V + GND; set `FRONT_BAR_COUNT=8`.

## Free pins

On the C3 Zero with the default Roby stack:

| GPIO | Notes |
|------|--------|
| 11–18 | Unused header GPIOs (verify silk before use) |
| 21 | UART RX / USB; avoid repurposing |
| Left TSOP | `IR_RECV_LEFT_PIN=-1` — GPIO available if you add a second fixed receiver |

Filled or special: 0–10, 19–20 (IR TX), 4 (speaker). Strapping / boot: prefer not to add new loads on **4**, **8**, **9** beyond the designed I2C bus.
