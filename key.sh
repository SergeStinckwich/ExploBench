#!/bin/sh

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/explore_beego
rosrun explore_beego key_img.py cmd_vel:=/beego/velocity image:=/beego/camera/image

