#!/usr/bin/env bash
# Enable rc.local-based autostart for the AirStack unified container on NVIDIA Orin.
# This will:
#  - Install /usr/local/bin/airstack-unified-autostart.sh (runs as user 'dtc')
#  - Create /etc/rc.local (or update) to invoke the autostart script on boot
#  - Install and enable rc-local.service with dependencies on network and docker

set -euo pipefail

if [[ $(id -u) -ne 0 ]]; then
  echo "Please run as root (use: sudo $0)" >&2
  exit 1
fi

USER_NAME="dtc"
USER_HOME="/home/${USER_NAME}"
REPO_ROOT="${USER_HOME}/georgia_dtc_ops_team_chiron_mrsd"
RUNNER_SCRIPT="${REPO_ROOT}/airstack/docker/run_unified.sh"
AUTOSTART_BIN="/usr/local/bin/airstack-unified-autostart.sh"
RC_LOCAL="/etc/rc.local"
RC_LOCAL_SVC="/etc/systemd/system/rc-local.service"

if [[ ! -f "${RUNNER_SCRIPT}" ]]; then
  echo "Could not find ${RUNNER_SCRIPT}. Is the repo cloned at ${REPO_ROOT}?" >&2
  exit 1
fi

echo "Installing ${AUTOSTART_BIN}"
cat > "${AUTOSTART_BIN}" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

# Log to user home to avoid permission issues
LOG_FILE="/home/dtc/airstack-unified.log"
mkdir -p "$(dirname "${LOG_FILE}")" || true
exec >>"${LOG_FILE}" 2>&1

echo "=== $(date) airstack-unified autostart begin ==="

# Wait for Docker daemon to be ready (up to ~2 minutes)
for i in {1..60}; do
  if systemctl is-active --quiet docker || docker info >/dev/null 2>&1; then
    break
  fi
  echo "[autostart] Waiting for docker... (${i})"
  sleep 2
done

# Defaults; customize by exporting these in the user environment if desired
export ROBOT_NAME="${ROBOT_NAME:-dtc_mrsd}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-70}"

REPO_ROOT="/home/dtc/georgia_dtc_ops_team_chiron_mrsd"
RUNNER_SCRIPT="${REPO_ROOT}/airstack/docker/run_unified.sh"

if [[ ! -x "${RUNNER_SCRIPT}" ]]; then
  echo "[autostart] ERROR: ${RUNNER_SCRIPT} not found or not executable"
  exit 0
fi

echo "[autostart] Launching unified container via ${RUNNER_SCRIPT}"
bash "${RUNNER_SCRIPT}" || true

echo "=== $(date) airstack-unified autostart end ==="
EOF

chmod 755 "${AUTOSTART_BIN}"
chown root:root "${AUTOSTART_BIN}"

echo "Configuring ${RC_LOCAL}"
if [[ -f "${RC_LOCAL}" ]]; then
  cp -a "${RC_LOCAL}" "${RC_LOCAL}.bak.$(date +%Y%m%d%H%M%S)"
fi

cat > "${RC_LOCAL}" <<'EOF'
#!/bin/sh -e
# rc.local - invoke AirStack unified autostart after boot

# Ensure we run as user 'dtc' in login shell (PATH, env)
su - dtc -c '/usr/local/bin/airstack-unified-autostart.sh &' || true

exit 0
EOF

chmod 755 "${RC_LOCAL}"

echo "Installing ${RC_LOCAL_SVC}"
cat > "${RC_LOCAL_SVC}" <<'EOF'
[Unit]
Description=/etc/rc.local Compatibility
ConditionPathExists=/etc/rc.local
After=network-online.target docker.service
Wants=network-online.target docker.service

[Service]
Type=forking
ExecStart=/etc/rc.local start
TimeoutSec=0
StandardOutput=journal+console
RemainAfterExit=yes
SysVStartPriority=99

[Install]
WantedBy=multi-user.target
EOF

chmod 644 "${RC_LOCAL_SVC}"
systemctl daemon-reload
systemctl enable rc-local.service

echo "Starting rc-local.service"
if ! systemctl start rc-local.service; then
  echo "rc-local.service failed to start now; it will run on next boot. Check: journalctl -u rc-local" >&2
fi

echo "Done. The unified container will autostart at boot."