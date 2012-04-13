#!/bin/sh

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/explore_beego
rosrun explore_beego dbg_actionlib.py
