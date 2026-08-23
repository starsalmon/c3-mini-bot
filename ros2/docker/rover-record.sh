#!/usr/bin/env bash
# Always-on rover ROS bag recorder — hourly files under ~/rover-bags.
set -eo pipefail
source /opt/ros/jazzy/setup.bash

OUT="${ROVER_BAG_DIR:-/bags}"
mkdir -p "$OUT"
HOURS="${ROVER_BAG_HOURS:-1}"

echo "rover-record → $OUT (${HOURS}h chunks)" >&2
while true; do
  STAMP=$(date -u +%Y-%m-%d_%H%M%S)
  DIR="$OUT/session_${STAMP}"
  echo "  recording $DIR" >&2
  # Record everything (topics + services). We can always ignore data later.
  # Exclude rosbag's own internal event topic to avoid self-noise.
  timeout "${HOURS}h" ros2 bag record -a --include-hidden-topics \
    --exclude-topics /events/write_split \
    -o "$DIR" \
    --qos-profile-overrides-path /opt/fleet/rover-record-qos.yaml \
    || true
  sleep 2
done
