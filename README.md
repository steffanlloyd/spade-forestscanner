# forest-scanner-ros1

ROS1 workspace for below-canopy drone.

To install
```bash
git clone --recurse-submodules git@gitlab.nibio.no:spade/forest-scanner-ros1.git

# Build docker
./scripts/init.sh
```
If you cloned it without the `--recurse-submodules` tag, you can fix with
```bash
git submodule update --init --recursive
```

A few scripts are defined to facilitate usage of the dockerfile.
```bash
./scripts/docker_build.sh # build the dockerfile
./scripts/docker_run.sh # run the dockerfile
./scripts/docker_stop.sh # stop the dockerfile
./scripts/docker_connect.sh # connect the dockerfile
```

Additionally, some scripts are defined to faciliate data collection:
```bash
# Triggers the launch files to record a dataset.
# Run with the screen command so that if the wifi connection is dropped,
# the script won't end.
# Note, need to start the docker first (with ./scripts/docker_run.sh).
./scripts/record_start.sh 

# Stops the recording
./scripts/record_stop.sh
```