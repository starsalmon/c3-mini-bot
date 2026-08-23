#!/usr/bin/env bash
# Run micro-ROS agent on your server VM (ROS 2 Jazzy).
# ESP firmware (c3_mini_wifi) connects over WiFi UDP port 8888 by default.
set -eo pipefail

PORT="${MICRO_ROS_AGENT_PORT:-8888}"

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi

echo "micro-ROS agent UDP4 port ${PORT}"
echo "Set AGENT_IP in platformio_private.ini to this host's LAN IP."
exec ros2 run micro_ros_agent micro_ros_agent udp4 --port "${PORT}"
