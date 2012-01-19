Beego Simulation
================

HOWTO
-----

1. install morse (cf. README)
2. run `morse beego.py` (cf. [slides](http://bit.ly/proteus2) )
3. in a new terminal, run `roscore`
4. ⌨ press "`P`" key in Blender 3D View to launch the simulation
5. in a new terminal, run: `rostopic pub -1 /Beego/Motion geometry_msgs/Twist [1,0,0] [0,0,1]`


_WARN_: Patch ROS Python3
-----------------------

You might need to patch ROS (due to a regression in Python3 compatibility):

    wget http://anr-proteus.github.com/slides/rospy3k.patch 
    sudo patch -p0 < rospy3k.patch

