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
from random import choice

class NextBestViewAlgorithm:
    """Abstract class for NBV algorithms"""
    candidates = None
    bestCandidate = None

    def chooseBestCandidate(self):
        abstract # Override me in derived class
        
    def chooseCandidatesOnFrontier(self):
        shouldBeImplemented

    def moveToBestCandidateLocation(self):
        shouldBeImplemented

    def loop(self, image):
        self.chooseCandidatesOnFrontier();
        self.chooseBestCandidate();
        self.moveToBestCandidateLocation()

    def __init__(self):
        rospy.init_node('NextBestViewAlgorithm')
        rospy.Subscriber('/map', OccupancyGrid, self.loop)

class RandomNBVAlgorithm(NextBestViewAlgorithm):
    """Move the robot to a randomly choosen candidates"""
    def chooseBestCandidate(self):
        self.bestCandidate = choice(candidates)

def main(argv):
    rospy.signal_shutdown("MainLoop")
    return 0

if __name__ == "__main__":
    sys.exit(main(sys.argv))
