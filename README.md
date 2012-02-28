ExploBench
==========

ExploBench is a tesbed to test/simulate exploration algorithms on top on the ROS framework.

MORSE Simulation
================

The morse folder contains Beego files for [MORSE](http://morse.openrobots.org/) 
simulation. MORSE is a generic simulator for academic robotics.

A video of MORSE simulation is available [here](http://youtube.com/embed/videoseries?list=PL289431A5B18BD997&rel=0&hd=1).

Before installing the MORSE simulation, you need to install ROS on Ubuntu Linux.
You can also run ROS and the simulation in a virtual machine like virtualBox, but the performance are quite low. You need to edit the morse/beego.py file in order to use wireframes for MORSE instead of 3D textures.

# Basic install with a script

Execute the morse/misc/install.sh

# Install by your own

On Ubuntu, you can install MORSE via this [package](http://ge.tt/20XDXJE):

    sudo apt-get install python3-dev python3.2-dev libsdl1.2debian python3-yaml
    sudo dpkg -i openrobots-morse_0.5_i386.deb
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


_WARN_: Patch ROS Python3
-----------------------
You might need to patch ROS (due to a regression in Python3 compatibility):

    wget http://anr-proteus.github.com/slides/rospy3k.patch 
    sudo patch -p0 < rospy3k.patch

# Install ExploBench
    git clone git://github.com/SergeStinckwich/ExploBench.git

# Run the simulation
1. cd ExploBench/morse
2. run `morse beego.py` (cf. [slides](http://bit.ly/proteus2) )
3. in a new terminal: cd ExploBench; source test.sh
4. press "`P`" key in Blender 3D View to launch the simulation
5. in a new terminal: cd ExploBench; source velexplo.sh
6. in order to view the sensors data, launch rviz: rosrun rviz rviz -d explore_beego/explore.vcg

# If you want to list all the topics from this simulation

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

