#!/usr/bin/env python
"""
usage:
rosrun explore_beego NextBestViewAlgorithm.py cmd:=/beego/velocity map:/explore/map
"""

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
from random
from perimeter import known_perimeter

class NextBestViewAlgorithm:
    """Abstract class for NBV algorithms"""
    candidates = None
    bestCandidate = None
    occupancy_grid = None

    def chooseBestCandidate(self):
        abstract # Override me in derived class
        
    def chooseCandidatesOnFrontier(self):
        data = self.occupancy_grid.data
        height = self.occupancy_grid.info.height
        width = self.occupancy_grid.info.width
        candidates = known_perimeter(data, width, height)
        
    def moveToBestCandidateLocation(self):
        """Use the navigation stack to move to the goal"""
        shouldBeImplemented
    
    def className(self):
        shouldBeImplemented

    def loop(self, occupancy_grid):
        self.occupancy_grid = occupancy_grid
        self.chooseCandidatesOnFrontier(self)
        self.chooseBestCandidate()
        self.moveToBestCandidateLocation()

    def __init__(self):
        rospy.init_node('NextBestViewAlgorithm:'+self.className)
        rospy.Subscriber('/map', OccupancyGrid, self.loop)
        
class RandomNBVAlgorithm(NextBestViewAlgorithm):
    """Move the robot to a randomly choosen candidates"""
    def chooseBestCandidate(self):
        self.bestCandidate = random.choice(candidates)
    
    def className(self):
        return('RandomNBVAlgorithm')

class MCDMPrometheeNBVAlgorithm(NextBextViewAlgorithm):
    def chooseBestCandidate(self):
        shouldBeImplemented

    def className(self):
        return('MCDMPrometheeNBVAlgorithm')

def main(argv):
    rospy.signal_shutdown("MainLoop")
    return 0

if __name__ == "__main__":
    sys.exit(main(sys.argv))
