cd /home/ros/ros1_ws/src/livox_ros_driver2/
cp /home/ros/ros1_ws/src/spade/scripts/build_livox.sh ./build.sh

# Prompt the user
read -p "Do you want to build fastlio? (y/n) " answer

source /opt/ros/melodic/setup.bash
./build.sh ROS1

cd /home/ros/ros1_ws
rosdep update

case ${answer:0:1} in
    y|Y )
        # Proceed normally
        ;;
    * )
        # Add your extra code here
        touch /home/ros/ros1_ws/src/livox_ros_driver/CATKIN_IGNORE
        touch /home/ros/ros1_ws/src/FAST_LIO/CATKIN_IGNORE
        touch /home/ros/ros1_ws/src/livox_to_pointcloud2/CATKIN_IGNORE
        ;;
esac

catkin_make