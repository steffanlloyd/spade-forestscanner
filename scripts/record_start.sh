#!/bin/bash
# Start a new detached screen session that runs the roslaunch command inside the Docker container
timestamp=$(date +%Y%m%d%H%M%S)
screen -S ros_session \
    -L -Logfile $(dirname "$0")/../rosbags/logs/ros_output_$timestamp.log \
    -dm bash -c 'docker exec -it forestscanner-ros1 bash -c "source /opt/ros/melodic/setup.bash && source /home/ros/ros1_ws/devel/setup.bash && roslaunch spade record.launch"'
