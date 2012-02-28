#!/bin/sh

sudo apt-get install build-essential python-setuptools subversion git-cvs mercurial python3-dev python3.2-dev ros-electric-ros-base ros-electric-exploration python3-yaml
# patch ROS Python 3 compatible
wget http://anr-proteus.github.com/slides/rospy3k.patch 
sudo patch -p0 < rospy3k.patch

if ! grep "source /opt/ros/electric/setup.bash" .bashrc ; then
    echo "# ROS install $(date +'%F %T UTC%z')" >> ~/.bashrc
    echo "source /opt/ros/electric/setup.bash" >> ~/.bashrc
fi

exit 0
# optionnal:

# install bosch_common
roslocate rosinstall bosch_common> bosch_common.rosinstall
mkdir -p ~/work/ros-addons
rosinstall ~/work/ros-addons /opt/ros/electric bosch_common.rosinstall 
echo "# ROS install $(date +'%F %T UTC%z')" >> ~/.bashrc
echo "source ~/work/ros-addons/setup.bash" >> ~/.bashrc
source ~/.bashrc
rosmake bosch_worlds

