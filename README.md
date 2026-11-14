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
| Optional 8× NeoPixel bar | Front status bar GPIO19 |

## Quick start

### Fleet firmware (normal operation)

```bash
cp platformio_private.ini.example platformio_private.ini
# Set WIFI_*, AGENT_IP=192.168.3.172, BOT_NAMESPACE=bot1

pio run -e c3_mini_wifi_ota_usb -t upload   # first USB flash only
pio run -e c3_mini_wifi_ota -t upload       # OTA — stop motion first (see OTA safety)
```

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

Tune wheels in `include/bot_pins.h` (`SERVO_STOP_US_*`, trims/inverts).
