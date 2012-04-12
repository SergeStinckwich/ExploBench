#!/bin/sh

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/explore_stage2
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/explore2
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/bosch_maps2
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/bosch_worlds2
roslaunch explore_stage2 explore.launch 

