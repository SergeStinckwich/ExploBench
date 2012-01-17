Beego Simulation
================

README
------

This folder contains Beego files for [MORSE](http://morse.openrobots.org/) 
simulation. MORSE is a generic simulator for academic robotics.

# Install

On Ubuntu, you can install MORSE via this [package](http://dl.free.fr/vOrT45L7P):

    sudo apt-get install python3-dev python3.2-dev libsdl1.2debian 
    sudo dpkg -i openrobots-morse-0.4.99-Linux.deb
    wget http://download.blender.org/release/Blender2.61/blender-2.61-linux-glibc27-i686.tar.bz2
    tar jxf blender-2.61-linux-glibc27-i686.tar.bz2
    export MORSE_BLENDER=$(pwd)/blender-2.61-linux-glibc27-i686/blender
    export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3/dist-packages


