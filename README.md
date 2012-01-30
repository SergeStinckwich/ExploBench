ExploBench
==========

A video of MORSE simulation is available [here](http://youtube.com/embed/I6m4DMM6bIM?rel=0&hd=1)

MORSE Simulation
================

The morse folder contains Beego files for [MORSE](http://morse.openrobots.org/) 
simulation. MORSE is a generic simulator for academic robotics.

Before installing the MORSE simulation, you need to install ROS.

# Basic install with a script

Execute the morse/misc/install.sh

# Install

On Ubuntu, you can install MORSE via this [package](http://dl.free.fr/vOrT45L7P):

    sudo apt-get install python3-dev python3.2-dev libsdl1.2debian 
    sudo dpkg -i openrobots-morse-0.4.99-Linux.deb
    wget http://download.blender.org/release/Blender2.61/blender-2.61-linux-glibc27-i686.tar.bz2
    tar jxf blender-2.61-linux-glibc27-i686.tar.bz2
    export MORSE_BLENDER=$(pwd)/blender-2.61-linux-glibc27-i686/blender
    export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3/dist-packages

if you want to make the export permanent, use:

    echo "export MORSE_BLENDER=$(pwd)/blender-2.61-linux-glibc27-i686/blender" >> ~/.bashrc
    echo "export PYTHONPATH=\$PYTHONPATH:/usr/local/lib/python3/dist-packages" >> ~/.bashrc
    source ~/.bashrc

to check your installation, use:

    morse check

see [MORSE README](https://github.com/laas/morse#readme)

[![Build Status](https://secure.travis-ci.org/pierriko/morse.png?branch=travis-upload)](http://travis-ci.org/pierriko/morse?branch=travis-upload)

# ROS

list of topic from this simulation:

    $ rostopic list -v

    Published topics:
     * /beego/odometry [nav_msgs/Odometry] 1 publisher
     * /beego/camera [sensor_msgs/Image] 1 publisher
     * /rosout [rosgraph_msgs/Log] 1 publisher
     * /beego/scan [sensor_msgs/LaserScan] 1 publisher
     * /rosout_agg [rosgraph_msgs/Log] 1 publisher

    Subscribed topics:
     * /beego/velocity [geometry_msgs/Twist] 1 subscriber
     * /rosout [rosgraph_msgs/Log] 1 subscriber

