#!/bin/bash
# Connect display, if there is a display
if [ -n "$DISPLAY" ]; then
    xhost +local:root
fi

# Check if the Docker container is running
if [ "$(docker ps -q -f name=forestscanner-ros1)" ]; then
    # Prompt the user
    read -p "The Docker container 'forestscanner-ros1' is already running. Do you want to restart it? (y/n) " answer
    case ${answer:0:1} in
        y|Y )
            # Stop the Docker container
            echo "Stopping docker..."
            docker stop forestscanner-ros1
            echo "Done."
            ;;
        * )
            # Exit the script
            echo "Exiting."
            exit
            ;;
    esac
fi

# Start the docker
echo "Starting docker forestscanner-ros1..."
docker run -itd \
  --name="forestscanner-ros1" \
  --rm \
  --privileged --network host --ipc host --user ros \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(dirname "$0")/../ros1_ws:/home/ros/ros1_ws \
  -v $(dirname "$0")/../rosbags:/home/ros/rosbags \
  -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  forestscanner-ros1:latest \
  tail -f /dev/null

echo "Done."

docker ps