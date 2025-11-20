#!/bin/bash

# Get the directory of the script
SCRIPT_DIR=$(dirname "$(realpath "$0")")

# Define the source and destination paths
SERVICE_FILE_SRC="$SCRIPT_DIR/../setup/setup-network.service.txt"
SERVICE_FILE_DEST="/etc/systemd/system/setup-network.service"

# Replace the {{{BASE PATH}}} placeholder with the script directory path
sudo sed "s|{{{BASE PATH}}}|$SCRIPT_DIR|g" "$SERVICE_FILE_SRC" | sudo tee "$SERVICE_FILE_DEST" > /dev/null

# Reload the systemd manager configuration
sudo systemctl daemon-reload

# Enable the service to start on boot
sudo systemctl enable setup-network.service

# Start the service immediately
sudo systemctl start setup-network.service

echo "Service setup-network.service has been installed and started."