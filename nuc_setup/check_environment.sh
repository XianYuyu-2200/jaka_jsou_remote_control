#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/env.bash" >/dev/null

mkdir -p "$LOG_ROOT"
OUT="$LOG_ROOT/system_$(date +%Y%m%d_%H%M%S).txt"

{
  echo "timestamp=$(date --iso-8601=seconds)"
  echo "hostname=$(hostname)"
  echo "kernel=$(uname -a)"
  lsb_release -a 2>/dev/null || true
  echo
  echo "ROS_DISTRO=$ROS_DISTRO"
  ros2 pkg prefix orbbec_camera
  ros2 pkg prefix jaka_driver
  echo
  echo "USB topology:"
  lsusb -t 2>/dev/null || true
  echo
  echo "Disk:"
  df -h "$ROOT"
  echo
  echo "GPU/rendering (optional):"
  glxinfo -B 2>/dev/null | sed -n '1,12p' || true
  echo
  echo "Orbbec devices:"
  ros2 run orbbec_camera list_devices_node || true
  echo
  echo "Available launch arguments:"
  ros2 launch orbbec_camera gemini210.launch.py --show-args || true
  ros2 launch jaka_driver robot_start.launch.py --show-args || true
} | tee "$OUT"

echo "Saved: $OUT"
