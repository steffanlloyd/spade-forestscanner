#!/usr/bin/env bash
#
# setup_remote_desktop_kms.sh
#
# Set up a headless Raspberry Pi (Ubuntu/Bookworm-like or Raspberry Pi OS) to run a real KMS desktop
# so NoMachine can attach to the *physical* session even with no monitor attached.
# This uses the KMS kernel arg "video=<connector>:<WxH>@<refresh>D" instead of xserver-xorg-video-dummy.
#
# Defaults:
#   HDMI_CONN=HDMI-A-2
#   RES_W=1920 RES_H=1080 REFRESH=60
#
# Optional env:
#   USERNAME=<login user>   # autologin user; auto-detected if unset
#   PURGE_DUMMY=yes         # purge xserver-xorg-video-dummy if present
#   REBOOT=yes              # reboot at the end to apply cmdline.txt change
#
set -euo pipefail

# --- Parameters ---
HDMI_CONN="${HDMI_CONN:-HDMI-A-2}"
RES_W="${RES_W:-1920}"
RES_H="${RES_H:-1080}"
REFRESH="${REFRESH:-60}"
PURGE_DUMMY="${PURGE_DUMMY:-no}"
REBOOT="${REBOOT:-no}"

echo "Installing NoMachine"
wget https://www.nomachine.com/free/arm/v8/deb -O nomachine.deb
dpkg -i nomachine.deb
rm nomachine.deb

# Figure out a target user for autologin (if enabled)
if [[ -n "${USERNAME:-}" ]]; then
  TARGET_USER="$USERNAME"
elif [[ -n "${SUDO_USER:-}" && "${SUDO_USER}" != "root" ]]; then
  TARGET_USER="${SUDO_USER}"
else
  TARGET_USER="$(id -un)"
fi

echo ">>> Config: ${HDMI_CONN} ${RES_W}x${RES_H}@${REFRESH}  User=${TARGET_USER}"

if [[ $EUID -ne 0 ]]; then
  echo "Please run as root (use sudo)."
  exit 1
fi

# --- Helpers ---
ensure_dir() { mkdir -p "$1"; }
write_file() { local path="$1"; shift; printf '%s\n' "$@" > "$path"; }

timestamp() { date +"%Y%m%d-%H%M%S"; }

# --- Packages (LightDM + XFCE + Xorg core) ---
export DEBIAN_FRONTEND=noninteractive
apt-get update -y
apt-get install -y \
  xorg xserver-xorg-core xinit \
  lightdm dbus-x11 \
  xfce4 xfce4-goodies

# Make LightDM the default display manager without prompting
write_file /etc/X11/default-display-manager "/usr/sbin/lightdm"

# --- Remove/disable any dummy X configuration ---
DUMMY_CONF=/etc/X11/xorg.conf.d/10-dummy.conf
if [[ -f "$DUMMY_CONF" ]]; then
  mv "$DUMMY_CONF" "${DUMMY_CONF}.disabled.$(timestamp)"
  echo ">>> Disabled existing dummy X config: ${DUMMY_CONF}"
fi

if [[ "$PURGE_DUMMY" == "yes" ]]; then
  if dpkg -l | awk '/^ii/ && $2=="xserver-xorg-video-dummy"{found=1} END{exit !found}'; then
    apt-get purge -y xserver-xorg-video-dummy || true
    apt-get autoremove -y || true
    echo ">>> Purged xserver-xorg-video-dummy"
  fi
fi

# --- LightDM configuration: set XFCE + autologin ---
ensure_dir /etc/lightdm/lightdm.conf.d
write_file /etc/lightdm/lightdm.conf.d/50-user-session.conf \
"[Seat:*]
user-session=xfce
"

groupadd -r autologin 2>/dev/null || true
gpasswd -a "${TARGET_USER}" autologin || true
write_file /etc/lightdm/lightdm.conf.d/50-autologin.conf \
"[Seat:*]
autologin-user=${TARGET_USER}
autologin-user-timeout=0
user-session=xfce
"
echo ">>> Enabled LightDM autologin for user: ${TARGET_USER}"

# --- KMS forced mode via kernel command line ---
# Choose the cmdline file location (Ubuntu/Bookworm uses /boot/firmware/cmdline.txt)
CMDLINE_FILE=""
if [[ -f /boot/firmware/cmdline.txt ]]; then
  CMDLINE_FILE="/boot/firmware/cmdline.txt"
elif [[ -f /boot/cmdline.txt ]]; then
  CMDLINE_FILE="/boot/cmdline.txt"
else
  echo "ERROR: Cannot find cmdline.txt (looked in /boot/firmware and /boot)."
  exit 1
fi

BACKUP="${CMDLINE_FILE}.bak.$(timestamp)"
cp -a "$CMDLINE_FILE" "$BACKUP"
echo ">>> Backed up ${CMDLINE_FILE} -> ${BACKUP}"

VIDEO_ARG="video=${HDMI_CONN}:${RES_W}x${RES_H}@${REFRESH}D"

# Read single line, strip any existing HDMI 'video=' tokens, append our VIDEO_ARG once
CURRENT=$(tr -d '\n' < "$CMDLINE_FILE")
# Remove any existing video=HDMI* tokens to avoid duplicates
CURRENT="$(echo "$CURRENT" | sed -E 's/[[:space:]]*video=HDMI[^[:space:]]+//g')"
# Also trim duplicate spaces
CURRENT="$(echo "$CURRENT" | sed -E 's/[[:space:]]+/ /g' | sed -E 's/^[[:space:]]+|[[:space:]]+$//g')"

# Append our new VIDEO_ARG
if [[ -n "$CURRENT" ]]; then
  NEWLINE="${CURRENT} ${VIDEO_ARG}"
else
  NEWLINE="${VIDEO_ARG}"
fi

# Ensure single line
printf '%s\n' "$NEWLINE" | tr -d '\n' > "$CMDLINE_FILE"
echo ">>> Set KMS forced mode in ${CMDLINE_FILE}: ${VIDEO_ARG}"

# --- Boot to graphical and enable LightDM ---
systemctl set-default graphical.target
systemctl enable --now lightdm

# --- Hints / optional reboot ---
echo ">>> Setup complete."
echo ">>> KMS forced mode active for ${HDMI_CONN} at ${RES_W}x${RES_H}@${REFRESH} (headless-friendly)."
echo ">>> A reboot is required for cmdline.txt changes to take effect."

if [[ "$REBOOT" == "yes" ]]; then
  echo ">>> Rebooting now..."
  systemctl reboot
else
  echo ">>> Reboot manually when convenient: sudo reboot"
fi
