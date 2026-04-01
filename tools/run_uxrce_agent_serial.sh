#!/usr/bin/env bash
set -e

DEV="${1:-/dev/ttyACM0}"
BAUD="${2:-921600}"

sudo MicroXRCEAgent serial --dev "$DEV" -b "$BAUD"
