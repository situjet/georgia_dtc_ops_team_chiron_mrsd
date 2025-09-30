#!/usr/bin/env bash
set -euo pipefail

# Simple wrapper to run the mavlink proxy with environment overrides.
# Defaults reflect the described network:
#   Onboard host: 10.3.1.32 (this machine)
#   QGC host:     10.3.1.96
#   Port:         14550
#
# You can override with environment variables:
#   AUTOPILOT_EP   (e.g. udp:0.0.0.0:14550 or tcp:127.0.0.1:5760)
#   QGC_HOST       (e.g. 10.3.1.96)
#   QGC_PORT       (e.g. 14550)
#   BIND_QGC_PORT  (optional source port; default empty => ephemeral)
#   AUTOPILOT_BAUD (if AUTOPILOT_EP is a serial device like /dev/ttyACM0, default 115200)
#
# Or pass extra args after --
#   ./run_mavlink_proxy.sh -- --print-stats

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY=${PYTHON:-python3}

AUTOPILOT_EP=${AUTOPILOT_EP:-/dev/serial/by-id/usb-CubePilot_CubeOrange_0-if00}
AUTOPILOT_BAUD=${AUTOPILOT_BAUD:-57600}
QGC_PORT=${QGC_PORT:-14550}
BIND_QGC_PORT=${BIND_QGC_PORT:-}

EXTRA_ARGS=()
if [[ -n "${BIND_QGC_PORT}" ]]; then
  EXTRA_ARGS+=("--bind-qgc-port" "${BIND_QGC_PORT}")
fi

# Forward all args after -- to python
PASSTHRU=()
FOUND_DOUBLE_DASH=0
for arg in "$@"; do
  if [[ "$arg" == "--" ]]; then
    FOUND_DOUBLE_DASH=1
    continue
  fi
  if [[ $FOUND_DOUBLE_DASH -eq 1 ]]; then
    PASSTHRU+=("$arg")
  fi
done

try_proxy() {
  local QGC_HOST="$1"
  echo "Attempting to start mavlink_proxy.py with QGC_HOST=$QGC_HOST"
  "$PY" "$SCRIPT_DIR/mavlink_proxy.py" \
    --autopilot "${AUTOPILOT_EP}" \
    --autopilot-baud "${AUTOPILOT_BAUD}" \
    --qgc "$QGC_HOST" \
    --qgc-port "${QGC_PORT}" \
    "${EXTRA_ARGS[@]}" \
    "${PASSTHRU[@]}"
}

set -x
try_proxy "172.20.2.207"
status=$?
if [[ $status -ne 0 ]]; then
  echo "First attempt failed, retrying with QGC_HOST=172.20.2.202"
  try_proxy "172.20.2.202"
fi
