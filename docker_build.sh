docker build \
    --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) \
    -t forestscanner-ros1:latest -f Dockerfile .