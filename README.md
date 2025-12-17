# forest-scanner-ros1

ROS1 workspace for below-canopy drone.

[Drone physical assembly and setup instructions here.](docs/DroneSetup.md)


## Setup
The steps below describe the setup of the SPADE drone. Note, this is already done if you ahve recieved the drone from SDU or NIBIO.

### Repository cloning
```bash
git clone --recurse-submodules git@github.com:steffanlloyd/spade-forestscanner.git
```
If you cloned it without the `--recurse-submodules` tag, you can fix with
```bash
git submodule update --init --recursive
```

### Computer Setup
To set up a raspberry pi for use, run the script. This script expects the Raspi to be installed with Ubuntu 24.04.
```
./scripts/raspi_setup.sh
```
This will:
 - Set up remote desktop with NoMachine
 - Set up SSH
 - Install docker
 - Remove bloatware in the install

You should now connect to the Raspberry Pi to wifi either through remote access or command line. The next step will dedicate the ethernet port to the LiDAR and it will not be possible to connect to the internet through it afterwards. Once complete, run:
```bash
./scripts/setup_remote_desktop.sh
```

To complete the setup of the LiDAR, you need to modify the IP address in the config file to match your sensor. Open `ros1_ws/src/spade/config/MID360_config.json`, then edit line 28, LiDAR ip address:
```
      "ip" : "192.168.1.1XX",
```
Change the last two digits of the IP address to match the last two digits of the serial number on the LiDAR.

### Docker setup
To set up the docker, just build it with the pre-made script:
```bash
./scripts/docker_build.sh
```
There are several scripts to help work with the docker:
```bash
./scripts/docker_run.sh # run the dockerfile
./scripts/docker_stop.sh # stop the dockerfile
./scripts/docker_connect.sh # connect the dockerfile
```
However, the most convenient method is to install a service on the machine to start the docker on the computers at startup. This avoids needing to manually start the docker in the field. This can be accomplished by running the script:
```bash
./scripts/setup_docker_at_startup.sh
```
After running this script, a service called `docker-run-forestscanner` will be installed and cause the docker to run at startup.

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

The ros workspace has 3 packages:
- `livox_ros_driver2`: The Livox sensor driver
- `livox_to_pointcloud2`: If you need to rebroadcast the custom lidar format to the more standard pointcloud2, this node will do that.
- `spade`: A custom SPADE folder, which just contains custom launch files for simplifing data collection.
    - `spade/launch/msg_MID360.launch`: Launches the LiDAR data acquisition.
    - `spade/launch/record.launch`: Launches the `msg_MID360` launch file, but also runs `mavros` to acquire the drone data, and creates a ROS bag in `/rosbags`.


## To trim a dataset:
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
