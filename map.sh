#!/bin/sh

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/explore_beego
rosrun explore_beego map.py cmd_vel:=/beego/velocity map:=/explore/map
