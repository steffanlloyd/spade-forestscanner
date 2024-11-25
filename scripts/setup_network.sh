#!/bin/bash
# This script sets a dedictated ip address on the ethernet port to 
# ensure the Lidar can connect properly. It automatically runs
# on startup via the /etc/rc.local mechanism.

# Wait for the eth0 interface to become available, with a timeout of 10 seconds
timeout=10
while ! ip link show eth0 > /dev/null 2>&1; do
    echo "Waiting for eth0 to be available"
    sleep 1
    ((timeout--))
    if [ $timeout -eq 0 ]; then
        echo "Timed out waiting for eth0 to become available"
        exit 1
    fi
done

ip link set eth0 up
ip addr add 192.168.1.5/24 dev eth0
ip route add 192.168.1.0/24 dev eth0
exit 0