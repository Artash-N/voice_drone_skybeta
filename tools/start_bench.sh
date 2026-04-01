#!/usr/bin/env bash
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_DIR"

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch voice_drone bench.launch.py
