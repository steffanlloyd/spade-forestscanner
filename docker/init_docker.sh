cd /home/ros/ros1_ws/src/livox_ros_driver2/
cp /home/ros/ros1_ws/src/spade/scripts/build_livox.sh ./build.sh

source /opt/ros/noetic/setup.bash
./build.sh ROS1

cd /home/ros/ros1_ws
rosdep update
catkin_make