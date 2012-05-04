#!/bin/sh

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/explore_beego
rosrun explore_beego wander.py cmd_vel:=/beego/velocity base_scan:=/beego/scan
