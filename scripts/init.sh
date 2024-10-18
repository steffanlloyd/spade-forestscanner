git submodule update --init --recursive

./docker_build.sh
./docker_run.sh
docker exec forestscanner-ros1:latest bash -c /home/ros/scripts/init.sh