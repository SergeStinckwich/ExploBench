Beego Simulation
================

TODO
----

  - Edit beego-robot.blend
  - see: [this slide](http://anr-proteus.github.com/slides/demo.html#slide25) or directly [from youtube](http://www.youtube.com/embed/videoseries?list=PLDC1FC34E5AC69429&hd=1&rel=0)

Docs
----

# ROS

    sudo apt-get install ros-electric-base ros-electric-bosch-common
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd ..)/explore_beego
    roslaunch explore_beego explore.launch 
    rosrun explore_beego remap.py from:=/beego/scan to:=/base_scan
    # if needed
    rostopic pub -1 /beego/velocity geometry_msgs/Twist [0.1,0,0] [0,0,0.1]

