# forest-scanner-ros1

ROS1 workspace for below-canopy drone.

[Drone physical assembly and setup instructions here.](docs/DroneSetup.md)


## To install repository
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

## Initiating data collection
If you have read the QAV guide produced by SDU, you do NOT need to configure the LiDAR ethernet. This is done automatically in this repository by running the scripts `./scripts/setup_networks.sh`. You don't even need to run this script even: it is set up to run on boot via the `/etc/rc.local` mechanism.

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
<<<<<<< HEAD
These scripts will create rosbags named by the timestamp, stored in the `rosbags` directory.

## Using QGroundControl
1. connect the battery and bind the transmitter with the receiver.
2. connect the telemetry radio to your pc and open up QGroundControl.
3. open up the vehicle setup by pressing the QGC logo as shown below.
4. calibrate the radio by following this guide: [https://docs.qgroundcontrol.com/master/en/qgc-userguide/setup_view/radio.html](https://docs.qgroundcontrol.com/master/en/qgc-userguide/setup_view/radio.html)

5. calibrate the sensers by following this guide: [https://docs.qgroundcontrol.com/master/en/qgcuser-
guide/setup_view/sensors_px4.html](https://docs.qgroundcontrol.com/master/en/qgcuser-
guide/setup_view/sensors_px4.html)
6. set up the flight mode by following this guide: [https://docs.qgroundcontrol.com/master/en/qgcuser-
guide/setup_view/flight_modes_px4.html](https://docs.qgroundcontrol.com/master/en/qgcuser-
guide/setup_view/flight_modes_px4.html)
7. Arm the vehicle by driving the right joystick to the lower right corner and you are ready to fly.
8. to disarm the vehicle, drive the left joystick to the lower left corner and push the kill swith.

## Reflashing the Jetson

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
=======

To trim a dataset:
```bash
rosbag info [bag].bag # Look at start and end time
rosbag filter input.bag output.bag "t.secs > 1731660634 and t.secs < 1731661242"
```

To test FASTLIO, start up from NoMachine:
```bash
# Restart docker from Nomachine
./scripts/docker_run.sh # press yes to restart
./scripts/docker_connect.sh # connect to docker
```
Then, in the docker:
```bash
sis # source
roslaunch spade mapping_mid360.launch
```
Then, replay the rosbag
```bag
rosbag play data.bag
```
>>>>>>> cf76509 (updates)
