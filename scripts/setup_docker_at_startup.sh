#!/usr/bin/env bash
set -euo pipefail

# Re-run as root if needed
if [[ $EUID -ne 0 ]]; then
  exec sudo -E bash "$0" "$@"
fi

# Resolve paths
THIS_SCRIPT="$(readlink -f "$0")"
SCRIPT_DIR="$(dirname "$THIS_SCRIPT")"
TARGET="${SCRIPT_DIR}/docker_run.sh"

# Sanity checks
if [[ ! -f "$TARGET" ]]; then
  echo "ERROR: ${TARGET} not found (it must live next to this installer)." >&2
  exit 1
fi

chmod +x "$TARGET"

SERVICE_NAME="docker-run-forestscanner"
UNIT_PATH="/etc/systemd/system/${SERVICE_NAME}.service"
WRAPPER="/usr/local/bin/${SERVICE_NAME}-wrapper.sh"

# Create a tiny wrapper so ExecStart doesn't choke on spaces in paths
cat > "$WRAPPER" <<EOF
#!/usr/bin/env bash
set -euo pipefail
exec "$TARGET" "\$@"
EOF
chmod +x "$WRAPPER"

# Create the systemd unit
cat > "$UNIT_PATH" <<EOF
[Unit]
Description=Run docker_run.sh at boot
Wants=network-online.target docker.service
After=network-online.target docker.service

# Use Wants= (not Requires=) so boot is never blocked if Docker/network is absent.

[Service]
Type=simple
User=spade
WorkingDirectory=${SCRIPT_DIR}
ExecStart=${WRAPPER}

# Don't block or fail the boot: failures just go to the journal
Restart=on-failure
RestartSec=10s
StartLimitIntervalSec=300
StartLimitBurst=5
StandardOutput=journal
StandardError=journal
NoNewPrivileges=yes
KillMode=process

[Install]
WantedBy=multi-user.target
EOF

# Activate
systemctl daemon-reload
systemctl enable "${SERVICE_NAME}.service"

# Start now (won't affect boot if this fails)
if ! systemctl start "${SERVICE_NAME}.service"; then
  echo "Note: service failed to start now; it will still run on next boot." >&2
fi

echo "Installed: ${UNIT_PATH}"
echo "Enabled : ${SERVICE_NAME}.service (runs at boot)"
echo
echo "Status:   systemctl status ${SERVICE_NAME}"
echo "Logs:     journalctl -u ${SERVICE_NAME} -f"
