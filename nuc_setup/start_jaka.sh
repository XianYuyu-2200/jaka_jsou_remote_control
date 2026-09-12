#!/usr/bin/env bash
set -eo pipefail

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <JAKA robot IP>" >&2
  exit 2
fi

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/env.bash" >/dev/null

exec ros2 launch jaka_driver robot_start.launch.py ip:="$1"
