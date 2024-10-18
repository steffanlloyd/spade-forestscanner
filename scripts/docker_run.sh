xhost +local:root

docker run -itd \
  --name="forestscanner-ros1" \
  --rm \
  --privileged --network host --ipc host --user ros \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(dirname "$0")/ros1_ws:/home/ros/ros1_ws \
  -v $(dirname "$0")/rosbags:/home/ros/rosbags \
  -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  forestscanner-ros1:latest \
  tail -f /dev/null

docker ps