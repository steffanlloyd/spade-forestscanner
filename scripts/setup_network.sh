#!/bin/bash

# Make eth0 purely manual, with fixed LiDAR IP
sudo nmcli connection modify eth0 ipv4.method manual ipv4.addresses "192.168.1.5/24" ipv4.gateway ""

# Optional: ignore IPv6 on eth0 (to avoid noise)
sudo nmcli connection modify eth0 ipv6.method ignore
sudo nmcli connection modify eth0 connection.autoconnect yes

# Bring it up
sudo nmcli connection up eth0
echo "Network setup complete: eth0 configured with static IP 192.168.1.5/24"