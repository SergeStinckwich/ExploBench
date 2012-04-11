#!/usr/bin/env python
"""
usage:
rosrun explore_beego NextBestViewAlgorithm.py XXXNBVAlgorithm
"""

import sys
import array
import random
import math
import threading
from abc import ABCMeta, abstractmethod
# ROS import
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('nav_msgs')
roslib.load_manifest('visualization_msgs')
roslib.load_manifest('move_base')
import rospy
import nav_msgs
import actionlib
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseAction

class NextBestViewAlgorithm(threading.Thread):
    """Abstract class for NBV algorithms"""
    __metaclass__ = ABCMeta
    # contains the different candidates positions for exploration
    # dictionnary: id (int32) -> pose (geometry_msgs/Pose)
    candidates = {}
    bestCandidate = None
    client = None
    pourcentageOfKnowEnv = 0
    maxPourcentageofCoverage = 0.90

    def __init__(self):
        threading.Thread.__init__(self)
        rospy.init_node("NBV%s"%self.className)
        rospy.Subscriber('visualization_marker', Marker, self.handle_markers)
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

    def run(self):
        # Waits until the action server has started up and started
        # listening for goals.
        self.client.wait_for_server()

        while not rospy.is_shutdown() and \
              (pourcentageUnkownEnv < maxPourcentageCoverage):
            self.chooseBestCandidate()
            self.moveToBestCandidateLocation()
            rospy.sleep(.2)

        print("exploration done !")

    @abstractmethod
    def chooseBestCandidate(self): pass

    def computePourcentageOfKnownEnv():
        nbOfUnknowCells = 0
        for eachCells in data:
            if (eachCells == -1):
                nbOfUnknownCells = nbOfUnknowCells + 1
        self.pourcentageOfKnownEnv = 1 - (nbOfUnknownCells / len(data))

    def moveToBestCandidateLocation(self):
        """Use the navigation stack to move to the goal"""

        if not bestCandidate:
            rospy.loginfo('No best candidate')
        else:
            rospy.loginfo('Move to best candidate')

            # Creates a goal to send to the action server.
            goal = MoveBaseGoal()
            goal.target_pose.pose.position.x = bestCandidate[0]
            goal.target_pose.pose.position.y = bestCandidate[1]

            # Sends the goal to the action server.
            self.client.send_goal(goal)

            # Waits for the server to finish performing the action.
            self.client.waitForResult()

    @property
    def className(self):
        return self.__class__.__name__

    def handle_markers(self, marker):
        if marker.action == Marker.DELETE:
            if marker.id in self.candidates:
                self.candidates.pop(marker.id)
        else:
            self.candidates[marker.id] = marker.pose

class RandomNBVAlgorithm(NextBestViewAlgorithm):
    """Move the robot to a randomly choosen candidate"""
    def chooseBestCandidate(self):
        self.bestCandidate = random.choice(candidates)

class MinimumLengthNBVAlgorithm(NextBestViewAlgorithm):
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
            pathLength += distanceBetweenPose(poses[index-1], poses[index])
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

class MaxQuantityOfInformationNBVAlgorithm(NextBestViewAlgorithm):
    def chhoseBestCandidate(self):
        shouldBeImplemented
    
    def distance(self, x1, y1, x2, y2):
        return sqrt(pow(x2-x1, 2) + pow(y2-y1, 2))

    def quantityOfNewInformation(self, candidate):
        #Compute the pourcentage of new information for a candidate
        # radius = How to have acess to the perception radius ???
        data = self.occupancy_grid.data
        numberOfUnknownCells = 0
        numberOfKnownCells = 0
        #Iteration in a square with center candidate
        for i in range(candidate.x-radius:candidate.x+radius):
            for j in range(candidate.y-radius:candidate.y+radius):
                #Test that we are in the disk
                if distance(self,i,j, candidate.x, candidate.y) < radius:
                    if (data[i][j] == -1):
                        numberOfUnknownCells = numberOfUnknownCells + 1
                    elif:
                        numberOfKnownCells = numberOfKnownCells + 1
        
        return numberOfUnknownCells / (numberOfKnownCells + numberOfUnknownCells)

class MCDMPrometheeNBVAlgorithm(NextBextViewAlgorithm):
    def chooseBestCandidate(self):
        shouldBeImplemented

def main(argv):
    #Use the Python reflection API to run the suitable NBV class
    classToLaunch = argv[1]
    print(classToLaunch)
    instance = getattr(sys.modules[__name__], classToLaunch)()
    instance.start()
    rospy.spin()
    return 0

if __name__ == "__main__":
    sys.exit(main(sys.argv))
