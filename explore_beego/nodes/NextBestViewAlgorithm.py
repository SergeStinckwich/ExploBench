#!/usr/bin/env python

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('nav_msgs')
roslib.load_manifest('geometry_msgs')
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
import wx
import sys
import threading
import array

class NextBestViewAlgorithm:
    """Abstract class for NBV algorithms"""

    def chooseBestCandidate(self):
        abstract # Override me in derived class
        
    def chooseCandidatesOnFrontier(self, image):
        shouldBeImplemented
 
    def loop(self, image):
        self.chooseCandidatesOnFrontier();
        self.chooseBestCandidate();
        self.moveToBestCandidateLocation()

    def __init__(self):
        rospy.init_node('NextBestViewAlgorithm')
        rospy.Subscriber('/map', OccupancyGrid, self.HandleImage)

def main(argv):

    rospy.signal_shutdown("MainLoop")
    return 0

if __name__ == "__main__":
    sys.exit(main(sys.argv))
