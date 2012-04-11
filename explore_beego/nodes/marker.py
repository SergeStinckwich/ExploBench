#!/usr/bin/env python
"""
marker
usage:
rosrun explore_beego marker.py marker:=/visualization_marker

http://ros.org/doc/api/visualization_msgs/html/msg/Marker.html
http://ros.org/wiki/rviz/DisplayTypes/Marker
"""

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('visualization_msgs')
import rospy
from visualization_msgs.msg import Marker

poses = {}

def handle(msg):
    #if not msg.id in poses: print(msg.pose)
    if msg.action == Marker.DELETE:
        if not msg.id in poses:
            print("error: Marker.DELETE: not a key: %i !!!!!!!!!!!!!!!"%msg.id)
        else:
            poses.pop(msg.id)
            print("info:  Marker.DELETE: %i <<<<<<<<<<<<<<<<<<<<<<<<<<"%msg.id)
    else:
        poses[msg.id] = msg.pose
    print(poses)

if __name__ == '__main__':
    rospy.init_node('marker')
    rospy.Subscriber('marker', Marker, handle)
    rospy.spin()