# Home Assistant — lab robots

Always-on `fleet-ha` on dockerhost publishes MQTT discovery (same broker as yoRadio).

## What shows up

| HA name | Robot | Notes |
|---------|-------|--------|
| **Rover** | LEGO rover (`/rover`) | Session + drive mode. Battery after the next rover OTA (`/rover/battery/voltage`). |
| **Roby** | c3-mini (`/bot1`) | Battery from firmware now. |
| **Tracked** | tracked-bot (`/bot2`) | Battery from firmware now. |
| **Lab robots** | fleet | **Stop all robots** + **Allow drive** |

Battery voltage/% are **retained** (voltage 2 decimal places, percent 1). The panel keeps the last reading when a bot is powered off. **Online** is live (ESP IMU/sonar/battery, ~8 s stale).

**Stop all** turns **Allow drive** off: zeros `cmd_vel`, ends rover session, and the brains stay halted. Turn **Allow drive** back on before Go / wander. Rover Go is ignored while halt is on.

## Wall panel (existing dashboard)

Add a **Bubble Card** pop-up (`#robots`). Entity IDs follow the device/unique_id prefixes Home Assistant actually created (`lab_robots_…` for the fleet, `lab_robot_…` per bot).

Paste `docs/ha-robots-dashboard.yaml` as a card on the dashboard that already has your other bubble pop-ups. Link it with hash `#robots` the same way as your other bubbles.

## dockerhost

`mqtt_secrets.env` next to compose (not in git). Sync copies it if present:

```bash
cd c3-mini-bot/ros2/docker
./sync-dockerhost.sh
ssh dockerhost 'cd ~/robot-fleet && docker compose up -d --build fleet-ha rover-brain tracked-brain fleet-brain'
```
