#!/bin/sh

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/explore_beego
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/explore_stage2
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/explore2
rosrun explore_beego NextBestViewAlgorithm.py MCDMPrometheeNBVAlgorithm
