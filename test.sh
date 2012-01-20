#!/bin/sh

# install
#sh morse/misc/install.sh
#sudo apt-get install ros-electric-base ros-electric-bosch-common

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/explore_beego
roscore &
roscorepid=$!
sleep 5
rosrun explore_beego remap_laser.py from:=/beego/scan to:=/base_scan &
remaplaserpid=$! # TODO put remap in launch
rosrun explore_beego remap_twist.py from:=/cmd_vel to:=/beego/velocity &
remaptwistpid=$! # TODO put remap in launch
rosrun explore_beego remap_odom.py from:=/beego/odometry to:=/odom &
remapodompid=$! # TODO put remap in launch
roslaunch explore_beego explore.launch 

# if needed
#rostopic pub -1 /beego/velocity geometry_msgs/Twist [0.1,0,0] [0,0,0.1]

echo "wait $roscorepid ... (Ctrl-C to interupt)"
wait $roscorepid
kill $remaplaserpid
kill $remaptwistpid
kill $remapodompid

