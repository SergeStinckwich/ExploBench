#!/usr/bin/env python
"""
remap odometry msg
usage:
rosrun explore_beego tf_map_odom.py odom:=/beego/odom
http://www.ros.org/wiki/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29	
"""

import roslib
roslib.load_manifest('tf')
roslib.load_manifest('rospy')
roslib.load_manifest('nav_msgs')
import tf
import rospy
from nav_msgs.msg import Odometry

def handler(msg):
    br = tf.TransformBroadcaster()
    p = msg.pose.pose.position
    position = (p.x, p.y, p.z)
    o = msg.pose.pose.orientation
    orientation = (o.x, o.y, o.z, o.w)
    br.sendTransform(position, orientation, rospy.Time.now(), "base_link", "map")

if __name__ == '__main__':
    rospy.init_node('tf_map_odom')
    rospy.Subscriber('odom', Odometry, handler)
    rospy.spin()

