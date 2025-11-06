#!/bin/bash
git submodule update --init --recursive

echo "Ignoring pointcloud2"
touch $(dirname "$0")/../ros1_ws/src/livox_to_pointcloud2/CATKIN_IGNORE

echo "Setting up ethernet service"
$(dirname "$0")/install_network_setup_service.sh

echo "Building docker"
$(dirname "$0")/docker_build.sh

echo "Running docker"
$(dirname "$0")/docker_run.sh