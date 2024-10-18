git submodule update --init --recursive

$(dirname "$0")/docker_build.sh
$(dirname "$0")/docker_run.sh
docker exec forestscanner-ros1 bash -c /home/ros/scripts/init.sh