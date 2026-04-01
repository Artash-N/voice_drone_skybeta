#!/usr/bin/env bash
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DIR="$WS_DIR/src"
BRANCH="${PX4_MSGS_BRANCH:-release/1.15}"

mkdir -p "$SRC_DIR"

if [ -d "$SRC_DIR/px4_msgs/.git" ]; then
  echo "px4_msgs already exists at $SRC_DIR/px4_msgs"
  echo "if you want a different branch, remove it first"
  exit 0
fi

git clone -b "$BRANCH" https://github.com/PX4/px4_msgs.git "$SRC_DIR/px4_msgs"

echo "done"
echo "cloned px4_msgs branch: $BRANCH"
