#!/usr/bin/env bash
set -e

PORT="${1:-8888}"
MicroXRCEAgent udp4 -p "$PORT"
