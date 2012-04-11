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

# contains the different candidates positions for exploration
# dictionnary: id (int32) -> pose (geometry_msgs/Pose)
candidates = {}

def handle(msg):
    if msg.action == Marker.DELETE:
        if msg.id in candidates:
            candidates.pop(msg.id)
    else:
        candidates[msg.id] = msg.pose

if __name__ == '__main__':
    rospy.init_node('marker')
    rospy.Subscriber('marker', Marker, handle)
    rospy.spin()