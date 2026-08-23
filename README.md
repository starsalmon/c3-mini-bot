# c3-mini-bot

Tiny ROS 2 bot: **ESP32-C3 Zero** on the body, **brains on your server VM** (or any machine running `micro_ros_agent`).

## Hardware

| Part | Role |
|------|------|
| ESP32-C3 Zero | micro-ROS client, WiFi or USB serial |
| 2× FS90R | Continuous rotation wheel servos |
| 1× FS90 | Pan servo for ultrasonic |
| HC-SR04 | Ultrasonic range |

## Pins — yes, you have enough

| GPIO | Use |
|------|-----|
| **0** | Left wheel servo signal |
| **1** | Right wheel servo signal |
| **3** | Pan servo signal |
| **5** | Ultrasonic TRIG |
| **6** | Ultrasonic ECHO |
| **10** | Onboard status LED (optional) |

**Avoid:** GPIO4 (flash/JTAG strap), GPIO8–9 (boot/I²C labels — fine later for IMU). GPIO20/21 = USB serial console.

**Power:** Run all three servos from **5 V** (≥2 A). Share GND with the C3. Signal wires go to GPIO above.

## micro-ROS on ESP32?

**Yes.** The C3 runs a **thin client** (motors + sonar + topics). Your VM runs:

- `micro_ros_agent` (bridge to ROS 2)
- Python nodes (`mini_wander.py`, etc.)

The robot does **not** need a Pi on board — only WiFi (or USB) to the agent.

## Quick start

### 1. Autonomous on the ESP (no VM, no WiFi) — **start here**

```bash
cd c3-mini-bot
pio run -e c3_autonomous -t upload
pio device monitor
```

On boot it **pan-scans**, then drives forward. It re-scans every ~2.5 s and when something is close (~22 cm): **reverse → turn toward the clearest angle → cruise**.

Tune in `src/autonomous_main.cpp`: `STOP_M`, `SLOW_M`, `CRUISE_LIN`, pan angles.

### 2. Bench test (serial keys, no auto)

```bash
pio run -e c3_bench -t upload
pio device monitor
```

Serial keys: `f` `b` `l` `r` `s` stop, `g` sonar read, `p` then degrees for pan.

### 3. WiFi + VM brain (later)

```bash
cp platformio_private.ini.example platformio_private.ini
# Edit SSID, password, AGENT_IP (your VM LAN IP)

pio run -e c3_mini_wifi -t upload
```

On the VM:

```bash
bash ros2/start_agent.sh
# another terminal:
source /opt/ros/jazzy/setup.bash
python3 ros2/mini_wander.py
```

### 3. USB serial agent (no WiFi)

```bash
pio run -e c3_mini_serial -t upload
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

## ROS topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/cmd_vel` | `geometry_msgs/Twist` | → ESP |
| `/sonar/range` | `sensor_msgs/Range` | ← ESP |
| `/sonar/pan_deg` | `std_msgs/Float32` | ← ESP (pan angle, future sweep) |

## Tune FS90R

If a wheel runs backward, swap trims in `drive_servos.cpp` or swap servo wires. Stop pulse is **1500 µs** (`SERVO_STOP_US` in `include/bot_pins.h`).
