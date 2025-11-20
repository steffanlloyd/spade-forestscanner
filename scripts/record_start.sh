#!/bin/bash
# Start a new detached screen session that runs the roslaunch command inside the Docker container

timestamp=$(date +%Y%m%d%H%M%S)
SESSION_NAME=ros_session
LOG_DIR="$(dirname "$0")/../rosbags/logs"
CONTAINER_NAME=forestscanner-ros1

mkdir -p "$LOG_DIR"

# Start roslaunch in a detached screen session
screen -S "$SESSION_NAME" \
    -L -Logfile "$LOG_DIR/ros_output_$timestamp.log" \
    -dm bash -c 'docker exec -it forestscanner-ros1 bash -c "source /opt/ros/noetic/setup.bash && source /home/ros/ros1_ws/devel/setup.bash && roslaunch spade record.launch"'

echo "Recording starting in background (screen session: $SESSION_NAME)..."

# Now verify required topics appear
REQUIRED_TOPICS=(
  "/livox/imu"
  "/livox/lidar"
  "/mavros/global_position/local"
)

TIMEOUT=20   # total time in seconds
INTERVAL=2   # check every 2 seconds
START_TIME=$(date +%s)

echo "Waiting for required topics to become available..."


while true; do
    NOW=$(date +%s)
    ELAPSED=$((NOW - START_TIME))

    if (( ELAPSED >= TIMEOUT )); then
        echo "ERROR: Required topics did not appear within ${TIMEOUT}s."
        echo "Stopping screen session '$SESSION_NAME'."
        # Kill the screen session (and thus the roslaunch inside it)
        screen -S "$SESSION_NAME" -X quit || true
        exit 1
    fi

    # Get topic list from inside the container
    TOPICS=$(docker exec "$CONTAINER_NAME" bash -lc \
        'source /opt/ros/noetic/setup.bash && source /home/ros/ros1_ws/devel/setup.bash && rostopic list 2>/dev/null' \
        2>/dev/null || echo "")

    # If we couldn't talk to ROS at all, just wait and retry
    if [[ -z "$TOPICS" ]]; then
        sleep "$INTERVAL"
        continue
    fi

    ALL_PRESENT=true
    for t in "${REQUIRED_TOPICS[@]}"; do
        if ! grep -q "^${t}\b" <<<"$TOPICS"; then
            ALL_PRESENT=false
            break
        fi
    done

    if $ALL_PRESENT; then
        echo "All required topics are active:"
        for t in "${REQUIRED_TOPICS[@]}"; do
            echo "  - $t"
        done
        echo "Recording confirmed running. Logs: $LOG_DIR/ros_output_$timestamp.log"
        break
    fi

    sleep "$INTERVAL"
done