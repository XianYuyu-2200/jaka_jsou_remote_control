#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/env.bash" >/dev/null

mkdir -p "$BAG_ROOT"
NAME="${1:-test_$(date +%Y%m%d_%H%M%S)}"

echo "First run 'ros2 topic list' and adjust TOPICS if needed."
TOPICS=(
  /camera/color/image_raw
  /camera/depth/image_raw
  /camera/color/camera_info
  /camera/depth/camera_info
  /camera/imu
  /jaka_driver/joint_position
  /jaka_driver/tool_position
  /jaka_driver/robot_states
)

ros2 bag record -s mcap -o "$BAG_ROOT/$NAME" "${TOPICS[@]}"
