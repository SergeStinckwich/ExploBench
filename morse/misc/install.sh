#!/bin/sh

cookie=$(mktemp)
url="http://dl.free.fr/fLBWg6yBM/openrobots-morse-0.4.99-Linux.deb"
echo "downloading morse package..."
wget --quiet --keep-session-cookies --save-cookies $cookie $url -O /dev/null
wget --quiet --load-cookies $cookie $url &
wgetpid=$!
echo "install morse dependencies:"
sudo apt-get install python3-dev python3.2-dev libsdl1.2debian python3-yaml python-setuptools
echo "waiting for $url ..."
wait $wgetpid
rm $cookie
url="http://download.blender.org/release/Blender2.61/blender-2.61-linux-glibc27-i686.tar.bz2"
echo "downloading blender..."
wget --quiet $url &
wgetpid=$!
sudo dpkg -i openrobots-morse-0.4.99-Linux.deb
echo "waiting for $url ..."
wait $wgetpid
tar jxf blender-2.61-linux-glibc27-i686.tar.bz2
echo "# MORSE install $(date +'%F %T UTC%z')" >> ~/.bashrc
echo "export MORSE_BLENDER=$(pwd)/blender-2.61-linux-glibc27-i686/blender" >> ~/.bashrc
echo "export PYTHONPATH=\$PYTHONPATH:/usr/local/lib/python3/dist-packages" >> ~/.bashrc
source ~/.bashrc
morse check

