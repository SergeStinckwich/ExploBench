#!/usr/bin/env python
"""
usage:
rosrun explore_beego lasergrid.py laser:=/scan

http://code.google.com/p/cs225turtle/source/browse/trunk/project2/local_obstacles/src/local_obstacles.cpp
"""

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('nav_msgs')
roslib.load_manifest('sensor_msgs')
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

SIZE = 550
RESOLUTION = 0.1

class LaserGrid(object):
    def __init__(self):
        rospy.init_node('lasergrid')
        self.og_pub = rospy.Publisher('test/map', OccupancyGrid)
        rospy.Subscriber('laser', LaserScan, self.handle_laser)
    def handle_laser(msg):
        og = OccupancyGrid
        og.header.frame_id = "/map"
        og.header.stamp = rospy.Time.now()
        og.info.resolution = RESOLUTION
        og.info.width = SIZE
        og.info.height = SIZE
        og.data.resize(SIZE * SIZE)
        self.og_pub.publish(og)
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    lg = LaserGrid()
    lg.run()