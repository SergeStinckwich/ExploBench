#!/bin/sh

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/explore_beego
rosrun explore_beego remap_twist.py from:=/cmd_vel to:=/beego/velocity 

