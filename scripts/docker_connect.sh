# Connect display, if there is a display
if [ -n "$DISPLAY" ]; then
    xhost +local:root
fi

docker exec -it forestscanner-ros1 /bin/bash