#!/usr/bin/env python
"""
remap odometry msg
usage:
rosrun explore_beego remap_odom.py from:=/beego/odom to:=/odom
"""

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('nav_msgs')
import rospy
from nav_msgs.msg import Odometry

def handler(msg):
    topic.publish(msg)

if __name__ == '__main__':
    rospy.init_node('RemapOdometry')
    topic = rospy.Publisher('to', Odometry)
    rospy.Subscriber('from', Odometry, handler)
    rospy.spin()

