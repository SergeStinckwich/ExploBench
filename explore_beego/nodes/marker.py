#!/usr/bin/env python
"""
marker
usage:
rosrun explore_beego marker.py marker:=/visualization_marker
"""

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('visualization_msgs')
import rospy
from visualization_msgs.msg import Marker

poses = {}

def handle(msg):
    #if not msg.id in poses: print(msg.pose)
    poses[msg.id] = msg.pose
    print(poses)

if __name__ == '__main__':
    rospy.init_node('marker')
    rospy.Subscriber('marker', Marker, handle)
    rospy.spin()