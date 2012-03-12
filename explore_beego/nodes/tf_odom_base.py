#!/usr/bin/env python
"""
broadcast odometry transformation
usage:
rosrun explore_beego tf_odom_base.py odom:=/beego/odom
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
    position = (p.x, p.y, 0) # XXX (p.x, p.y, p.z)
    o = msg.pose.pose.orientation
    orientation = (0, 0, o.z, o.w) # XXX (o.x, o.y, o.z, o.w)
    br.sendTransform(position, orientation, rospy.Time.now(), "/base_footprint", "/odom")

if __name__ == '__main__':
    rospy.init_node('tf_odom_base')
    rospy.Subscriber('odom', Odometry, handler)
    rospy.spin()

