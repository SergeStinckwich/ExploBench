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
import actionlib
import math

from perimeter import known_perimeter

class NextBestViewAlgorithm:
    """Abstract class for NBV algorithms"""
    candidates = None
    bestCandidate = None
    occupancy_grid = None
    client = None
    pourcentageOfKnowEnv = 0
    maxPourcentageofCoverage = 0.90

    def chooseBestCandidate(self):
        abstract # Override me in derived class
    
    def computePourcentageOfKnownEnv(self):
        nbOfUnknowCells = 0
        for eachCells in data:
            if (eachCells == -1):
                nbOfUnknownCells = nbOfUnknowCells + 1
        self.pourcentageOfKnownEnv = 1 - (nbOfUnknownCells / len(data))

    def chooseCandidatesOnFrontier(self):
        data = self.occupancy_grid.data
        height = self.occupancy_grid.info.height
        width = self.occupancy_grid.info.width
        #Update % of known env vs unknown env
        self.computePourcentageOfKnownEnv()
        #Compute candidates on the frontier
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
        abstract

    def loop(self, occupancy_grid):
        #Loop until we cover a large amount of the env
        if (pourcentageUnkownEnv < maxPourcentageCoverage):
            self.occupancy_grid = occupancy_grid
            self.chooseCandidatesOnFrontier(self)
            self.chooseBestCandidate(self)
            self.moveToBestCandidateLocation(self)
        
    def __init__(self):
        rospy.init_node('NextBestViewAlgorithm:'+self.className(self))
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
    """Exploration algorithm that use the criteria of length of the minimum collision-free path to candidate"""
    
    def distanceBetweenPose(self, pose1, pose2):
        """Compute the euclidian distance between 2 poses"""
        return sqrt(pow(pose2.position.x-pose1.position.x, 2) + pow(pose2.position.y-pose1.position.y, 2))

    def computePathLength(self, plan):
        """Compute the length path with the poses of the plan"""
        poses = plan.poses
        pathLength = 0
        #Iteration among along the poses in order to compute the length
        for index in range(1, len(poses)):
            pathLength = pathLength + distanceBetweenPose(self, poses[index-1], poses[index])
        return pathLength

    def chooseBestCandidate(self):
        #Wait for the availability of this service
        rospy.wait_for_service('make_plan')
        #Get a proxy to execute the service
        make_plan = rospy.ServiceProxy('make_plan', nav_msgs.srv.GetPlan)

        bestCandidate = None
        shortestLength = 0
        firstCandidate = True
        #Find the candidate with the shortest path
        for eachCandidate in candidates: 
            #Execute service for each candidates
            plan = make_plan(eachCandidate)
            #Compute the length of the path
            pathLength = computePathLength(plan)
            #Set the shortestPath for the first candidate
            if firstCandidate:
                shortestPath = pathLength
                bestCandidate = eachCandidate
                firstCandidate = False
            #If this is the shortest path until now, we found a new bestCandidate
            if pathLength < shortestLength:
                bestCandidate = eachCandidate
                shortestLength = pathLength

    def className(self):
        return('MinimumLengthNBVAlgorithm')

class MaxQuantityOfInformationNBVAlgorithm(NextBestViewAlgorithm):
    def className(self):
        return('MaxQuantityOfInformationNBVAlgorithm')
    
    def chhoseBestCandidate(self):
        shouldBeImplemented
    
    def distance(self, x1,y1,x2,y2):
        return sqrt(pow(x2-x1, 2) + pow(y2-y1, 2))

    def quantityOfNewInformation(self, candidate):
        #Compute the pourcentage of new information for a candidate
        # radius = How to have acess to the perception radius ???
        data = self.occupancy_grid.data
        numberOfUnknownCells = 0
        numberOfKnownCells = 0
        #Iteration in a square with center candidate
        for i in range(candidate.x-(radius/2):candidate.x+(radius/2)):
            for j in range(candidate.y-(radius/2):candidate.y+(radius/2)):
                #Test that we are in the disk
                if distance(self,i,j, candidate.x, candidate.y) < radius:
                    if (data[i][j] == -1):
                        numberOfUnknownCells = numberOfUnknownCells + 1
                    elif:
                        numberOfKnownCells = numberOfKnownCells + 1
        
        return numberOfUnknownCells / (numberOfKnownCells + numberOfUnkwonCells)

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
