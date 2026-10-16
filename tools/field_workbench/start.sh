#!/usr/bin/env bash
# Run on Ubuntu 22.04 directly or through SSH. Never starts CAN, Nav2 or motor control.
set -e
source /opt/ros/humble/setup.bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ -f "$ROOT/install/can_fix/setup.bash" ]]; then
  source "$ROOT/install/can_fix/setup.bash"
fi
cd "$ROOT"
exec python3 tools/field_workbench/server.py --host 127.0.0.1 --port "${FWMINI_WORKBENCH_PORT:-8765}"
