#!/usr/bin/env python
"""
remap twist msg
usage:
rosrun explore_beego remap_twist.py from:=/cmd_vel to:=/beego/velocity
"""

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('geometry_msgs')
import rospy
from geometry_msgs.msg import Twist

def handler(msg):
    topic.publish(msg)

if __name__ == '__main__':
    rospy.init_node('RemapTwist')
    topic = rospy.Publisher('to', Twist)
    rospy.Subscriber('from', Twist, handler)
    rospy.spin()

