#!/bin/sh

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/explore_beego
rosrun explore_beego marker.py marker:=/visualization_marker
