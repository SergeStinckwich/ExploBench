#!/bin/sh

url="http://open.ge.tt/1/files/20XDXJE/0/blob"
echo "downloading morse package..."
wget --quiet $url -O openrobots-morse_0.5_i386.deb &
wgetpid=$!
echo "install morse dependencies:"
sudo apt-get install python3-dev python3.2-dev libsdl1.2debian python3-yaml python-setuptools
url2="http://download.blender.org/release/Blender2.61/blender-2.61-linux-glibc27-i686.tar.bz2"
echo "downloading blender..."
wget --quiet $url2 &
wgetpid2=$!
echo "waiting for $url ..."
wait $wgetpid
sudo dpkg -i openrobots-morse_0.5_i386.deb
echo "waiting for $url2 ..."
wait $wgetpid2
tar jxf blender-2.61-linux-glibc27-i686.tar.bz2
echo "# MORSE install $(date +'%F %T UTC%z')" >> ~/.bashrc
echo "export MORSE_BLENDER=$(pwd)/blender-2.61-linux-glibc27-i686/blender" >> ~/.bashrc
echo "export PYTHONPATH=\$PYTHONPATH:/usr/local/lib/python3/dist-packages" >> ~/.bashrc
source ~/.bashrc
morse check

