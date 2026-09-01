#!/usr/bin/env bash
# Run from repo root OR anywhere — syncs fleet stack to dockerhost.
set -eo pipefail

HOST="${DOCKERHOST:-dockerhost}"
DEST="${ROBOT_FLEET_DIR:-robot-fleet}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS2_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ROVER_ROS2_DIR="$(cd "$SCRIPT_DIR/../../../lego-rover-ros2" && pwd)"

echo "→ $HOST:~/$DEST/"

ssh "$HOST" "mkdir -p ~/$DEST/cal"
scp \
  "$SCRIPT_DIR/docker-compose.yml" \
  "$SCRIPT_DIR/fastdds_udp.xml" \
  "$SCRIPT_DIR/ros.sh" \
  "$SCRIPT_DIR/rover-record.sh" \
  "$SCRIPT_DIR/rover-record-qos.yaml" \
  "$SCRIPT_DIR/topic-once.sh" \
  "$SCRIPT_DIR/swarm-explore.sh" \
  "$SCRIPT_DIR/setup-dockerhost.sh" \
  "$ROS2_DIR/mini_wander.py" \
  "$ROS2_DIR/beacon_chase.py" \
  "$ROS2_DIR/explore_then_chase.py" \
  "$ROS2_DIR/fleet_steering.py" \
  "$ROS2_DIR/analyze_drive_log.py" \
  "$ROS2_DIR/imu_motion_test.py" \
  "$ROVER_ROS2_DIR/rover_brain.py" \
  "$ROVER_ROS2_DIR/explore_controller.py" \
  "$ROVER_ROS2_DIR/cmd_vel_slew.py" \
  "$ROVER_ROS2_DIR/rover_proximity.py" \
  "$ROVER_ROS2_DIR/hallway_follow.py" \
  "$ROVER_ROS2_DIR/analyze_rover_telem.py" \
  "$ROVER_ROS2_DIR/rover_pan_validate.py" \
  "$ROVER_ROS2_DIR/rover_qos.py" \
  "$ROVER_ROS2_DIR/rover_twist.py" \
  "$HOST:~/$DEST/"
scp "$SCRIPT_DIR/cal/"*.sh "$HOST:~/$DEST/cal/" 2>/dev/null || true

ssh "$HOST" "chmod +x ~/$DEST/ros.sh ~/$DEST/rover-record.sh ~/$DEST/topic-once.sh ~/$DEST/swarm-explore.sh \
  ~/$DEST/cal/*.sh 2>/dev/null || true"
echo "Done. On dockerhost:"
echo "  cd ~/$DEST && docker compose up -d"
echo "  ./ros.sh topic list"
echo ""
echo "Bench/cal tools (not normal operation): ~/$DEST/cal/"
