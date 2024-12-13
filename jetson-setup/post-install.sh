#!/bin/bash

# Install some prerequesites that are handy
echo "Adding helpful software"
sudo apt-get update
sudo apt install nano tree screen nvidia-jetpack
echo "Done installing software."

# Install docker
echo "Installing and setting up docker"
bash $(dirname "$0")/../scripts/docker_install.sh
echo "Done installing docker."

# Install nomachine
# Best to look up a more recent updated link from https://downloads.nomachine.com/linux/?distro=Arm&id=30
echo "Installing NoMachine"
wget https://download.nomachine.com/download/8.14/Arm/nomachine_8.14.2_1_arm64.deb
sudo dpkg -i nomachine_8.14.2_1_arm64.deb
rm nomachine_8.14.2_1_arm64.deb
echo "Done installing NoMachine."

# Setup SSH
echo "Setting up SSH..."
mkdir ~/.ssh
touch ~/.ssh/authorized_keys
chmod 700 ~/.ssh
chmod 600 ~/.ssh/authorized_keys
chown -R spade:spade ~/.ssh

# Setup wifi drivers
echo "Installing wifi drivers..."
git clone https://github.com/aircrack-ng/rtl8188eus.git
cd rtl8188eus
make && sudo make install
echo 'blacklist r8188eu' | sudo tee -a '/etc/modprobe.d/realtek.conf'
echo 'blacklist rtl8xxxu' | sudo tee -a '/etc/modprobe.d/realtek.conf'
cd ..
rm -rf rtl8188eus

echo "Done setup. Please reboot for wifi drivers to come into effect!"