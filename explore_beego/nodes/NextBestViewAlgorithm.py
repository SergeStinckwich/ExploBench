#!/usr/bin/env python
"""
usage:
rosrun explore_beego NextBestViewAlgorithm.py XXXNBVAlgorithm cmd:=/beego/velocity map:/explore/map
"""

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('nav_msgs')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('actionlib')
import rospy
import nav_msgs
from geometry_msgs.msg import Twist
import sys
import array
import random
from perimeter import known_perimeter
import actionlib

class NextBestViewAlgorithm:
    """Abstract class for NBV algorithms"""
    candidates = None
    bestCandidate = None
    occupancy_grid = None
    client = None

    def chooseBestCandidate(self):
        abstract # Override me in derived class
        
    def chooseCandidatesOnFrontier(self):
        data = self.occupancy_grid.data
        height = self.occupancy_grid.info.height
        width = self.occupancy_grid.info.width
        candidates = known_perimeter(data, width, height)
        
    def moveToBestCandidateLocation(self):
        """Use the navigation stack to move to the goal"""

        if bestCandidate = None:
            rospy.loginfo('No best candidate')
        elif:
            rospy.loginfo('Move to best candidate')
            
            # Creates a goal to send to the action server.
            goal = MoveBaseGoal()
            goal.target_pose.pose.position.x = bestCandidate[0]
            goal.target_pose.pose.position.y = bestCandidate[1]
            
            # Sends the goal to the action server.
            self.client.send_goal(goal)
    
            # Waits for the server to finish performing the action.
            self.client.waitForResult()
        
    def className(self):
        shouldBeImplemented

    def loop(self, occupancy_grid):
        self.occupancy_grid = occupancy_grid
        self.chooseCandidatesOnFrontier(self)
        self.chooseBestCandidate()
        self.moveToBestCandidateLocation()

    def __init__(self):
        rospy.init_node('NextBestViewAlgorithm:'+self.className)
        rospy.Subscriber('/map', nav_msgs.msg.OccupancyGrid, self.loop)
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        
        # Waits until the action server has started up and started
        # listening for goals.
        self.client.wait_for_server()

class RandomNBVAlgorithm(NextBestViewAlgorithm):
    """Move the robot to a randomly choosen candidate"""
    def chooseBestCandidate(self):
        self.bestCandidate = random.choice(candidates)
    
    def className(self):
        return('RandomNBVAlgorithm')

class MinimumLengthNBVAlgorithm(NextBEstViewAlgorithm):
    """Length of the minimum collision-free path to candidate"""
    
    def chooseBestCandidate(self):
        #Wait for the availability of this service
        rospy.wait_for_service('make_plan')
        #Get a proxy to execute the service
        make_plan = rospy.ServiceProxy('make_plam', nav_msgs.srv.GetPlan)

        bestCandidate = None
        shortestLength = 0
        #Find the candidate with the shortest path
        for eachCandidate in candidates: 
            #Execute service for each candidates
            plan = make_plan(eachCandidate)
            #Compute the length of the path
            pathLenght = shouldbeImplemented
            #If the shortest path until now
            if pathLength < shortestLength:
                bestCandidate = eachCandidate
                shortestLength = pathLength

    def className(self):
        return('MinimumLengthNBVAlgorithm')

class MCDMPrometheeNBVAlgorithm(NextBextViewAlgorithm):
    def chooseBestCandidate(self):
        shouldBeImplemented

    def className(self):
        return('MCDMPrometheeNBVAlgorithm')

def main(argv):
    rospy.signal_shutdown("MainLoop")
    #Use the Python reflection API to run the suitable NBV class
    classToLaunch = argv[3]
    getattr(sys.modules[__name__], classToLaunch)()
    return 0

if __name__ == "__main__":
    sys.exit(main(sys.argv))
