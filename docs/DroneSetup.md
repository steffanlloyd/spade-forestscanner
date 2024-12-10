# Drone Setup

Here are some brief instructions to set up the drone, upon delivery.

![Drone Image](drone.jpg)

## Reassemble drone
1. Connect legs. Legs are in the plastic bag with the screws. One leg goes on each wing as in the attached image.
2. Connect the lidar assembly to the top of the computer with the four bolts already screwed into the computer case. Connect the LiDAR cable to the LiDAR, and double check that the power connector has the right polarity (red to red, black to black).
3. When ready for flight, the battery is strapped to the bottom.

## Computer setup

First, we need to connect the device to wifi. To do this, remove the ethernet cable to the LiDAR from the PC, and connect it to a network you control.

Then, SSH into the device. Username is `spade`, password is `spade`.

Once in, connect to your wifi network:
```bash
nmcli device wifi connect 'NETWORK_NAME' password 'PASSWORD' ifname wlan0
```
where you replace `NETWORK_NAME` and `PASSWORD` with your network's credentials.

Verify this connection was successful by ssh'ing into the device again over wifi. If successful, reconnect the LiDAR cable. Setup is complete.

If any problems, contact Steffan Lloyd at steffan.lloyd@nibio.no.