#!/usr/bin/env bash
# Source this file in every terminal used for the Gemini 215 / JAKA tests.
set -e

source /opt/ros/humble/setup.bash
source "$HOME/codex/codex-jaka_mini_2/orbbec_ws/install/setup.bash"
source "$HOME/codex/codex-jaka_mini_2/jaka_ros2/install/setup.bash"

export JAKA_MINI2_ROOT="$HOME/codex/codex-jaka_mini_2"
export ORBBEC_WS="$JAKA_MINI2_ROOT/orbbec_ws"
export JAKA_WS="$JAKA_MINI2_ROOT/jaka_ros2"
export BAG_ROOT="$JAKA_MINI2_ROOT/bags"
export LOG_ROOT="$JAKA_MINI2_ROOT/logs"

echo "ROS_DISTRO=$ROS_DISTRO"
echo "ORBBEC_WS=$ORBBEC_WS"
echo "JAKA_WS=$JAKA_WS"
