#!/usr/bin/env python
"""
remap laser msg
usage:
rosrun explore_beego remap.py from:=/beego/scan to:=/scan
"""

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib_msgs')
import rospy
from actionlib_msgs.msg import GoalStatus
from actionlib_msgs.msg import GoalStatusArray

def handle(msg):
    for goal in msg.status_list:
        if goal.status != GoalStatus.ACTIVE and goal.status != GoalStatus.SUCCEEDED:
            print(msg)

if __name__ == '__main__':
    rospy.init_node('dbg_actionlib')
    rospy.Subscriber('move_base/status', GoalStatusArray, handle)
    rospy.spin()
