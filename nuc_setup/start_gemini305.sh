#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/env.bash" >/dev/null

exec ros2 launch orbbec_camera gemini_301_series.launch.py \
  depth_registration:=true \
  enable_point_cloud:=false \
  enable_colored_point_cloud:=false \
  enable_frame_drop_log:=true \
  frame_timestamp_csv_file:="$LOG_ROOT/camera_timestamps_gemini305_$(date +%Y%m%d_%H%M%S).csv" \
  color_width:=1280 \
  color_height:=800 \
  color_fps:=30 \
  depth_width:=1280 \
  depth_height:=800 \
  depth_fps:=30 \
  "$@"
