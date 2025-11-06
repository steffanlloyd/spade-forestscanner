#!/bin/bash

set -e

if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
fi

if [ -f "/home/ros/ros1_ws/devel/setup.bash" ]; then
    source /home/ros/ros1_ws/devel/setup.bash
fi

echo "Provided arguments: $@"

exec $@
