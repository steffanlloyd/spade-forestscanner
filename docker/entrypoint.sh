#!/bin/bash

set -e

source /opt/ros/melodic/setup.bash
source /home/ros/ros1_ws/devel/setup.bash
source /home/ros/ros1_ws/devel/setup.sh

echo "Provided arguments: $@"

exec $@
