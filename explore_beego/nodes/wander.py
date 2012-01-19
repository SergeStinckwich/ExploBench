#!/usr/bin/env python
"""
wander avoid obstacles
usage:
rosrun explore_beego wander.py laser:=/beego/scan cmd:=/beego/velocity
"""

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('sensor_msgs')
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def handle_sick(msg):
    mid = len(msg.ranges) // 2
    cmd = Twist()
    # halt if an object is less than 2m in a 30deg angle
    halt=False
    for distance_to_object in msg.ranges[mid-15:mid+15]:
        if distance_to_object < 2:
            halt=True
            break
    if halt:
        # we go to the highest-range side scanned
        if sum(msg.ranges[:mid]) > sum(msg.ranges[mid:]):
            cmd.angular.z = -1
        else:
            cmd.angular.z = 1
    else:
        cmd.linear.x = 1
    # publish twist
    topic.publish(cmd)

if __name__ == '__main__':
    rospy.init_node('wander')
    topic = rospy.Publisher('cmd', Twist)
    rospy.Subscriber('laser', LaserScan, handle_sick)
    rospy.spin()

