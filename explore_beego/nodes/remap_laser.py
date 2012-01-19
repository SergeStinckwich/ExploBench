#!/usr/bin/env python
"""
remap laser msg
usage:
rosrun explore_beego remap.py from:=/beego/scan to:=/scan
"""

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('sensor_msgs')
import rospy
from sensor_msgs.msg import LaserScan

def handle_laser(msg):
    topic.publish(msg)

if __name__ == '__main__':
    rospy.init_node('RemapLaserScan')
    topic = rospy.Publisher('to', LaserScan)
    rospy.Subscriber('from', LaserScan, handle_laser)
    rospy.spin()

