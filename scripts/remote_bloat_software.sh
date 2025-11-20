#!/usr/bin/env bash
#
# ubuntu-minimalize.sh
# Remove common "bloat" apps from a default Ubuntu GNOME install.
# - Removes: LibreOffice, Thunderbird, Rhythmbox, most GNOME games, a few extras
# - Keeps:   Firefox
#
# Usage:
#   sudo bash ubuntu-minimalize.sh
#

set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "Please run as root, e.g.: sudo $0"
  exit 1
fi

if ! command -v apt-get >/dev/null 2>&1; then
  echo "This script is intended for Debian/Ubuntu systems using apt."
  exit 1
fi

# List of packages (or apt patterns) to purge.
# Feel free to edit this list to your taste.
PKGS=(
  # Office
  "libreoffice*"

  # Mail
  "thunderbird*"

  # Music / media
  "rhythmbox"
  "rhythmbox-data"

  # GNOME games (pretty much all the usual suspects)
  "aisleriot"
  "five-or-more"
  "four-in-a-row"
  "gnome-2048"
  "gnome-chess"
  "gnome-klotski"
  "gnome-mahjongg"
  "gnome-mines"
  "gnome-nibbles"
  "gnome-robots"
  "gnome-sudoku"
  "gnome-taquin"
  "gnome-tetravex"
  "hitori"
  "iagno"
  "lightsoff"
  "quadrapassel"
  "swell-foop"
  "tali"

  # Other "nice-to-have" desktop stuff (comment out if you want to keep)
  "cheese"          # webcam app
  "shotwell"        # photo manager
  "simple-scan"     # document scanner
  "totem"           # Videos
  "transmission-gtk"
  "transmission-common"
  "deja-dup"        # backup tool
)

echo "The following packages/patterns will be purged if installed:"
printf '  %s\n' "${PKGS[@]}"
echo
echo "NOTE:"
echo "  - Firefox is NOT in this list and will be kept."
echo "  - Some meta-packages (like ubuntu-desktop) *might* be removed if they"
echo "    depend strictly on these apps. That's usually harmless, but be aware."
echo

read -rp "Continue and purge these packages? [y/N] " answer
case "$answer" in
  [Yy]* ) ;;
  * ) echo "Aborted."; exit 0 ;;
esac

echo "Updating package lists..."
apt-get update

echo "Purging selected packages (ok if some are 'not installed')..."
apt-get purge -y "${PKGS[@]}" || true

echo "Removing automatically-installed dependencies..."
apt-get autoremove -y --purge

echo "Cleaning apt cache..."
apt-get clean

echo "Done. You may want to reboot to fully apply the changes."
