#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/env.bash" >/dev/null

exec ros2 launch orbbec_camera gemini210.launch.py \
  depth_registration:=true \
  enable_accel:=true \
  enable_gyro:=true \
  enable_frame_drop_log:=true \
  frame_timestamp_csv_file:="$LOG_ROOT/camera_timestamps_$(date +%Y%m%d_%H%M%S).csv" \
  "$@"
