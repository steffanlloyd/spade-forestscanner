# forest-scanner-ros1

ROS1 workspace for below-canopy drone.

[Drone assembly and setup instructions here.](docs/DroneSetup.md)

To install
```bash
git clone --recurse-submodules git@gitlab.nibio.no:spade/forest-scanner-ros1.git

# Build docker and make workspace
./scripts/init.sh
```
After this, you should be able to just call `catkin_make` normally.

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

If you need to reflash the Jetson (to install a new version, or if you break it): Follow the instructions in [./jetson-setup/README.md](./jetson-setup/README.md).

## Docker Usage

The docker images contains the following folders:
- `ros1_ws`: The ros workspace
- `rosbags`: When you run the `record_start.sh` and `record_stop.sh` scripts, rosbags will be stored here.
- `libraries`: Stores the Livox SDK2 folder, and any others that might be added.

The ros workspace has 3 packages:
- `livox_ros_driver2`: The Livox sensor driver
- `livox_to_pointcloud2`: If you need to rebroadcast the custom lidar format to the more standard pointcloud2, this node will do that.
- `spade`: A custom SPADE folder, which just contains custom launch files for simplifing data collection.
    - `spade/launch/msg_MID360.launch`: Launches the LiDAR data acquisition.
    - `spade/launch/record.launch`: Launches the `msg_MID360` launch file, but also runs `mavros` to acquire the drone data, and creates a ROS bag in `/rosbags`.