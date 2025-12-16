#!/bin/bash
# Start a new detached screen session that runs the roslaunch command inside the Docker container

timestamp=$(date +%Y%m%d%H%M%S)
SESSION_NAME=ros_session
LOG_DIR="$(dirname "$0")/../rosbags/logs"
CONTAINER_NAME=forestscanner-ros1
SERVICE_NAME=docker-run-forestscanner.service

# Ensure the docker-run-forestscanner service has finished before starting
<<<<<<< HEAD
SERVICE_NAME=docker-run-forestscanner.service
=======
>>>>>>> e60b415 (Fix)
SERVICE_STATUS=$(systemctl is-active "$SERVICE_NAME" 2>/dev/null || true)
if [[ "$SERVICE_STATUS" == "active" || "$SERVICE_STATUS" == "activating" ]]; then
    echo "Service $SERVICE_NAME is not finished yet (current state: $SERVICE_STATUS)."
    echo "Aborting: wait for $SERVICE_NAME to finish before starting recording."
    exit 1
fi

# Ensure there is no existing recording screen session
if screen -list | grep -q "[.]${SESSION_NAME}[[:space:]]"; then
    echo "A screen session named '${SESSION_NAME}' already exists."
    echo "Aborting to avoid starting a duplicate recording session."
    exit 1
fi

# Make log directory if it doesn't exist
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
<<<<<<< HEAD
  "/mavros/global_position/local"
)

TIMEOUT=20   # total time in seconds
INTERVAL=2   # check every 2 seconds
=======
  "/mavros/global_position/global"
)

TIMEOUT=20        # total time in seconds for topic discovery
INTERVAL=2        # check every 2 seconds
MSG_TIMEOUT=5     # timeout per topic for message check

>>>>>>> e60b415 (Fix)
START_TIME=$(date +%s)

echo "Waiting for required topics to become available..."

<<<<<<< HEAD

=======
>>>>>>> e60b415 (Fix)
while true; do
    NOW=$(date +%s)
    ELAPSED=$((NOW - START_TIME))

    if (( ELAPSED >= TIMEOUT )); then
        echo "ERROR: Required topics did not appear within ${TIMEOUT}s."
        echo "Stopping screen session '$SESSION_NAME'."
<<<<<<< HEAD
        # Kill the screen session (and thus the roslaunch inside it)
=======
>>>>>>> e60b415 (Fix)
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
<<<<<<< HEAD
        if ! grep -q "^${t}\b" <<<"$TOPICS"; then
=======
        if ! grep -q "^${t}\$" <<<"$TOPICS"; then
>>>>>>> e60b415 (Fix)
            ALL_PRESENT=false
            break
        fi
    done

    if $ALL_PRESENT; then
<<<<<<< HEAD
        echo "All required topics are active:"
        for t in "${REQUIRED_TOPICS[@]}"; do
            echo "  - $t"
        done
        echo "Recording confirmed running. Logs: $LOG_DIR/ros_output_$timestamp.log"
=======
        echo "All required topics exist."
>>>>>>> e60b415 (Fix)
        break
    fi

    sleep "$INTERVAL"
<<<<<<< HEAD
done
=======
done

# --------------------------------------------
# Verify messages are actually being published
# --------------------------------------------
echo ""
echo "Verifying messages are being published on each topic..."

FAILED_TOPICS=()

for topic in "${REQUIRED_TOPICS[@]}"; do
    echo -n "  Checking ${topic}... "
    
    # Use timeout + rostopic echo to wait for at least one message
    # rostopic echo -n 1 exits after receiving one message
    if docker exec "$CONTAINER_NAME" bash -lc \
        "source /opt/ros/noetic/setup.bash && source /home/ros/ros1_ws/devel/setup.bash && timeout ${MSG_TIMEOUT} rostopic echo -n 1 '${topic}' >/dev/null 2>&1"; then
        echo "OK (receiving messages)"
    else
        echo "FAILED (no messages within ${MSG_TIMEOUT}s)"
        FAILED_TOPICS+=("$topic")
    fi
done

echo ""

# Handle failures
if [[ ${#FAILED_TOPICS[@]} -gt 0 ]]; then
    echo "ERROR: The following topics are not publishing messages:"
    for t in "${FAILED_TOPICS[@]}"; do
        echo "  - $t"
    done
    echo ""
    echo "Stopping screen session '$SESSION_NAME'."
    screen -S "$SESSION_NAME" -X quit || true
    exit 1
fi

echo "All required topics are active and publishing:"
for t in "${REQUIRED_TOPICS[@]}"; do
    echo "  - $t"
done
echo ""
echo "Recording confirmed running. Logs: $LOG_DIR/ros_output_$timestamp.log"
>>>>>>> e60b415 (Fix)
