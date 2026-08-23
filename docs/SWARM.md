# Swarm / fleet

Fleet IDs (for humans/scripts — ROS namespaces differ):

| Fleet ID | Robot | Nickname | ROS namespace |
|----------|-------|----------|---------------|
| **bot0** | LEGO rover | — | `/rover/…` (Pi + ESP) |
| **bot1** | c3-mini | Roby | `/bot1/…` |

One **micro-ROS agent** on dockerhost bridges ESP32 minis into the same ROS 2 DDS graph (`bot1`, `bot2`, …). The LEGO rover runs its own stack on the Pi.

## c3-mini-bot / Roby (bot1, ESP32)

- Firmware: `c3_mini_wifi` with `BOT_NAMESPACE` in `platformio_private.ini`
- Topics (example `bot1`):
  - `/bot1/cmd_vel`
  - `/bot1/sonar/range`, `/bot1/sonar/pan_deg`
  - `/bot1/imu`
  - `/bot1/ir/detected`
  - `/bot1/battery/voltage`

## LEGO rover (bot0, Pi onboard)

Yes — keep the Pi. It joins the swarm as a **full ROS 2 node** on the same `ROS_DOMAIN_ID` (default `0`). No need to remove the Pi for GPIO, camera, or heavier compute.

Suggested layout:

| Robot | Fleet ID | Compute | Namespace | Agent |
|-------|----------|---------|-----------|-------|
| LEGO rover | **bot0** | Pi (+ ESP helper) | `rover` | Pi runs native ROS 2 (no agent required for Pi nodes) |
| c3-mini (Roby) | **bot1** | ESP32 only | `bot1` | dockerhost UDP 8888 |

On the Pi:

```bash
export ROS_DOMAIN_ID=0
# optional: point CycloneDDS at dockerhost if cross-subnet discovery is flaky
ros2 run …  # nodes under /rover/…
```

ESP on the rover can either:

1. Stay a micro-ROS client → Pi-local agent or dockerhost agent, namespace `rover`, or
2. Stay a serial/I2C peripheral while the Pi publishes `/rover/cmd_vel`, `/rover/…`

Fleet brains on dockerhost: **`docker compose up -d`** starts the agent, **fleet-brain** (explore→chase), and **fleet-shell** (ROS CLI). No profiles or extra scripts for normal operation.

Optional dev-only brains (do not run alongside fleet-brain):

```bash
docker compose stop fleet-brain
BOT_NS=bot1 docker compose --profile wander up -d   # wander only
BOT_NS=bot1 docker compose --profile chase up -d     # IR chase only
```

Only one process should publish a given `cmd_vel` at a time.

### Battery (`/bot1/battery/voltage`)

Published at 1 Hz from firmware (report only, no shutdown). **Run ROS CLI on dockerhost** — cross-subnet DDS from your Mac often won’t see bot topics:

```bash
cd ~/robot-fleet
docker compose up -d    # agent + fleet-brain + fleet-shell
./ros.sh topic hz /bot1/battery/voltage    # ~1 Hz when bot is online
./ros.sh topic echo /bot1/battery/voltage --once   # may warn once before first sample
./topic-once.sh /bot1/battery/voltage 12           # preferred — waits up to 12s
```

`topic hz` showing ~1.0 means the topic **is** publishing. The `does not appear to be published yet` warning on `echo --once` is a ROS 2 startup race (subscriber connects before the next 1 Hz sample). Use `topic-once.sh` or let `hz` run a few seconds first.

## Rover + mini together

| Robot | Brain | Drives |
|-------|-------|--------|
| c3-mini (`bot1`) | dockerhost `fleet-brain` | `/bot1/cmd_vel` |
| LEGO rover | **Pi** `autonomous_explore.py` | `/cmd_vel` |

The rover is **not** on dockerhost — Pi runs its own micro-ROS serial agent and `go_auto.sh`. The mini uses dockerhost UDP agent. They do not share one brain; they coordinate by **IR beacon** (rover TX) + **TSOP on mini pan** (bot RX).

```bash
# dockerhost — full stack (default)
cd ~/robot-fleet
docker compose up -d

# Pi — rover wanders with IR beacon (ESP firmware)
ssh lego-rover 'bash ~/lego-rover-ros2/go_auto.sh'
```

Mini wanders on sonar until it sees the rover beacon; then it chases. Rover keeps exploring on its own.

## Quick start (explore + chase)

**Prerequisites (once per session):**

| | Check |
|---|--------|
| **Mini** | Powered, WiFi, OTA firmware with `BOT_NAMESPACE=bot1`, `AGENT_IP` → dockerhost |
| **dockerhost** | `microros-agent` on UDP **8888** |
| **Rover** | Pack switch **ON**, ESP flashed (`s3_tdisplay_microros_rover` or OTA), Pi `rover-agent.service` running |
| **IR** | Rover **GPIO21** beacon forward; mini **GPIO7** TSOP on pan head |

**1 — Mini brain (from Mac):**

```bash
bash ~/Documents/cursor-esp32/c3-mini-bot/ros2/docker/go_swarm.sh
```

Or on dockerhost:

```bash
cd ~/robot-fleet && ./swarm-explore.sh
```

**2 — Rover brain:**

```bash
bash ~/Documents/cursor-esp32/lego-rover-ros2/sync_to_pi.sh   # after script changes
ssh lego-rover 'bash ~/lego-rover-ros2/go_swarm.sh'
```

Or press **Go** on the rover if `rover-main.service` is already in standby.

**3 — Watch it work (dockerhost):**

```bash
cd ~/robot-fleet
./ros.sh topic echo /bot1/ir/detected
# false while pan-scanning, true when rover beacon in view → explore_then_chase switches to chase
```

**Stop:** Pi — Go button or Ctrl+C. dockerhost — `docker compose stop fleet-brain` (or `docker compose down` for everything).

Motor/speed calibration tools live in `~/robot-fleet/cal/` and are **not** part of normal operation — use USB `c3_bench` for wheel tuning on battery.

## IR receiver (single TSOP on pan servo)

| TSOP leg | Connect |
|----------|---------|
| 1 OUT | GPIO **7** |
| 2 GND | GND |
| 3 VCC | **3.3 V** |

Lens toward you, legs down: **1 = OUT, 2 = GND, 3 = VCC**.

Firmware sweeps the pan servo when no IR is seen and holds the bearing when it is. `beacon_chase` steers from `/bot1/ir/right` + `/bot1/sonar/pan_deg` (default). Set `IR_DUAL=1` if you add a second fixed TSOP later.

## IR LED beacon (rover transmit)

Rover ESP **GPIO21** — 38 kHz, `IR_TX_DEFAULT_ON=1` in rover firmware (always on at boot).

| ESP32 | Connect |
|-------|---------|
| **GPIO21** | 1 kΩ → NPN base; IR LED + resistor from 5 V → collector |

Point forward on the rover — mini chases this, not the mini’s own GPIO20 TX.

For swarm chase — rover beacon is separate hardware; mini IR TX stays **off** (`IR_TX_FOLLOW_CMD=0`).
