docker run -it \
  --name="forestscanner-ros1" \
  --privileged --network host --ipc host --user ros \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/spade/ros1/ros1_ws:/home/ros/ros1_ws \
  -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  forestscanner-ros1:latest /bin/bash