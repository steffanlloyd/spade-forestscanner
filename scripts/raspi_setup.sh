#!/bin/bash
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y openssh-server git curl tree nano screen

git submodule update --init --recursive

SCRIPT_DIR=$(dirname "$(realpath "$0")")

# Setup SSH
echo "Setting up SSH..."
mkdir ~/.ssh
touch ~/.ssh/authorized_keys
chmod 700 ~/.ssh
chmod 600 ~/.ssh/authorized_keys
chown -R spade:spade ~/.ssh

echo "Building docker"
$(dirname "$0")/setup_remote_desktop.sh
$(dirname "$0")/docker_install.sh
