#!/bin/sh

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/explore_beego
#rosrun explore_beego NextBestViewAlgorithm.py MCDMPrometheeNBVAlgorithm
#rosrun explore_beego NextBestViewAlgorithm.py MinimumLengthNBVAlgorithm
rosrun explore_beego NextBestViewAlgorithm.py GBLNBVAlgorithm base_scan:=/beego/scan odom:=/beego/odometry
