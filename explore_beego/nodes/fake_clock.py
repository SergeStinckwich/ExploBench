#!/usr/bin/env python
"""
publish rospy.Time.now on /clock @ 10Hz
usage:
rosrun explore_beego fake_clock.py
"""

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('rosgraph_msgs')
import rospy
from rosgraph_msgs.msg import Clock

if __name__ == '__main__':
    rospy.init_node('fake_clock')
    msg = Clock()
    publisher = rospy.Publisher('/clock', Clock)
    while not rospy.is_shutdown():
        msg.clock = rospy.Time.now()
        publisher.publish(msg)
        rospy.sleep(.1)

