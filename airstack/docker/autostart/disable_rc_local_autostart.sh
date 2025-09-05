#!/usr/bin/env bash
# Disable rc.local-based autostart for the AirStack unified container.

set -euo pipefail

if [[ $(id -u) -ne 0 ]]; then
  echo "Please run as root (use: sudo $0)" >&2
  exit 1
fi

RC_LOCAL_SVC="/etc/systemd/system/rc-local.service"
RC_LOCAL="/etc/rc.local"
AUTOSTART_BIN="/usr/local/bin/airstack-unified-autostart.sh"

systemctl disable rc-local.service || true
systemctl stop rc-local.service || true

if [[ -f "${RC_LOCAL_SVC}" ]]; then
  rm -f "${RC_LOCAL_SVC}"
fi

if [[ -f "${RC_LOCAL}" ]]; then
  mv -f "${RC_LOCAL}" "${RC_LOCAL}.disabled"
fi

if [[ -f "${AUTOSTART_BIN}" ]]; then
  rm -f "${AUTOSTART_BIN}"
fi

systemctl daemon-reload

echo "Disabled rc.local autostart for AirStack."