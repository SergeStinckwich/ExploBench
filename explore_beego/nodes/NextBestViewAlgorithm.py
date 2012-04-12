#!/usr/bin/env python
"""
usage:
rosrun explore_beego NextBestViewAlgorithm.py XXXNBVAlgorithm

http://ros.org/doc/api/visualization_msgs/html/msg/Marker.html
http://www.ros.org/doc/api/move_base_msgs/html/msg/MoveBaseGoal.html
"""

import sys
import array
import random
import math
from abc import ABCMeta, abstractmethod
# ROS import
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('nav_msgs')
roslib.load_manifest('visualization_msgs')
roslib.load_manifest('move_base')
roslib.load_manifest('nav_msgs')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('sensor_msgs')
import rospy
import actionlib
from nav_msgs.srv import GetPlan
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import PyMCDA

class NextBestViewAlgorithm(object):
    """Abstract class for NBV algorithms"""
    __metaclass__ = ABCMeta
    # contains the different candidates positions for exploration
    # dictionnary: id (int32) -> pose (geometry_msgs/Pose)
    candidates = {}
    bestCandidate = None
    client = None
    pourcentageOfKnowEnv = 0
    maxPourcentageofCoverage = 0.90
    robot_pose = None
    occupancy_grid = None
    distance_traveled = 0.0
    robot_last_pose = None
    radius = None
    subscriber_laser_once = None

    def __init__(self):
        rospy.init_node("NBV%s"%self.className)
        rospy.Subscriber('visualization_marker', Marker, self.handle_markers)
        rospy.Subscriber('explore/map', OccupancyGrid, self.handle_occupancy_grid)
        rospy.Subscriber('odom', Odometry, self.handle_odom)
        sub_once = None
        self.subscriber_laser_once = rospy.Subscriber('base_scan', LaserScan, self.handle_laserscan)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def run(self):
        # Waits until the action server has started up and started
        # listening for goals.
        self.client.wait_for_server()

        while not rospy.is_shutdown() and \
              (self.pourcentageOfKnowEnv < self.maxPourcentageofCoverage):
            self.chooseBestCandidate()
            self.moveToBestCandidateLocation()
            rospy.sleep(.2)

        print("exploration done !")

    @abstractmethod
    def chooseBestCandidate(self): pass

    def computePourcentageOfKnownEnv():
        if not self.occupancy_grid:
            return 0
        data = self.occupancy_grid.data
        nbOfUnknownCells = 0
        for eachCells in data:
            if (eachCells == -1):
                nbOfUnknownCells = nbOfUnknowCells + 1
        self.pourcentageOfKnownEnv = 1 - (nbOfUnknownCells / len(data))

    def distanceBetweenPose(self, pose1, pose2):
        """Compute the euclidian distance between 2 poses"""
        return math.sqrt(pow(pose2.position.x-pose1.position.x, 2) +
                         pow(pose2.position.y-pose1.position.y, 2))

    def computePathLength(self, plan):
        """Compute the length path with the poses of the plan"""
        poses = plan.poses
        pathLength = 0
        #Iteration among along the poses in order to compute the length
        for index in range(1, len(poses)):
            pathLength += self.distanceBetweenPose(poses[index-1].pose, poses[index].pose)
        return pathLength

    def distance(self, x1, y1, x2, y2):
        return math.sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2))

    def quantityOfNewInformation(self, candidate):
        # Compute the pourcentage of new information for a candidate
        # TODO: get radius from laser scan topic
        if not self.occupancy_grid:
            return 0
        data = self.occupancy_grid.data
        width = self.occupancy_grid.info.width
        height = self.occupancy_grid.info.height
        resolution = self.occupancy_grid.info.resolution
        numberOfUnknownCells = 0.0
        numberOfKnownCells = 0.0
        origin_position = self.occupancy_grid.info.origin.position
        relative_radius = int(5 / resolution) # self.raduis not same of explore
        # 5 = laser range for explore, see explore_costmap.yaml
        relative_position_x = int((candidate.position.x - origin_position.x) /
                                   resolution)
        relative_position_y = int((candidate.position.y - origin_position.y) /
                                   resolution)

        # http://ros.org/doc/electric/api/nav_msgs/html/msg/OccupancyGrid.html
        # scale and origin
        #Iteration in a square with center candidate
        scan_min_x = max(relative_position_x - relative_radius, 0)
        scan_min_y = max(relative_position_y - relative_radius, 0)
        scan_max_x = min(relative_position_x + relative_radius, width)
        scan_max_y = min(relative_position_y + relative_radius, height)
        print("%i %i %i %i"%(scan_min_x,scan_min_y,scan_max_x,scan_max_y))

        for i in range(scan_min_x, scan_max_x):
            for j in range(scan_min_y, scan_max_y):
                #Test that we are in the disk
                if self.distance(i, j, relative_position_x, relative_position_y) < relative_radius:
                    data_pose = j * width + i
                    if (data[data_pose] == -1):
                        numberOfUnknownCells += 1
                    else:
                        numberOfKnownCells += 1

        print("numberOfKnownCells: %i numberOfUnknownCells: %i"%(numberOfKnownCells, numberOfUnknownCells))
        return numberOfUnknownCells / (numberOfKnownCells + numberOfUnknownCells)

    def moveToBestCandidateLocation(self):
        """Use the navigation stack to move to the goal"""

        if not self.bestCandidate:
            rospy.loginfo('No best candidate')
        else:
            rospy.loginfo('Move to best candidate')

            # Creates a goal to send to the action server.
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.pose = self.bestCandidate
            print(goal)

            # Sends the goal to the action server.
            self.client.send_goal(goal)

            # Waits for the server to finish performing the action.
            self.client.wait_for_result() # timeout: rospy.Duration.from_sec(5.0)

    @property
    def className(self):
        return self.__class__.__name__

    def handle_markers(self, marker):
        if marker.action == Marker.DELETE:
            if marker.id in self.candidates:
                self.candidates.pop(marker.id)
        else:
            self.candidates[marker.id] = marker.pose

    def handle_occupancy_grid(self, msg):
        self.occupancy_grid = msg

    def handle_laserscan(self, msg):
        self.radius = msg.range_max
        self.subscriber_laser_once.unregister()
        self.subscriber_laser_once = None

    def handle_odom(self, msg):
        if not self.robot_last_pose:
            self.robot_last_pose = msg.pose.pose
            return 
        self.robot_pose = msg.pose.pose
        # diff
        self.distance_traveled += self.distanceBetweenPose(self.robot_pose, 
                                                    self.robot_last_pose)
        self.robot_last_pose = self.robot_pose

class RandomNBVAlgorithm(NextBestViewAlgorithm):
    """Move the robot to a randomly choosen candidate"""
    def chooseBestCandidate(self):
        if self.candidates.values():
            self.bestCandidate = random.choice(self.candidates.values())

class MinimumLengthNBVAlgorithm(NextBestViewAlgorithm):
    """Exploration algorithm that use the criteria of length
    of the minimum collision-free path to candidate"""

    def chooseBestCandidate(self):
        #Wait for the availability of this service
        rospy.wait_for_service('move_base/make_plan')
        #Get a proxy to execute the service
        make_plan = rospy.ServiceProxy('move_base/make_plan', GetPlan)

        self.bestCandidate = None
        shortestLength = 0
        firstCandidate = True
        start = PoseStamped()
        start.header.frame_id = "map"
        start.pose = self.robot_pose
        goal = PoseStamped()
        goal.header.frame_id = "map"
        tolerance = 0.0
        #Find the candidate with the shortest path
        for eachCandidate in self.candidates.values():
            #Execute service for each candidates
            goal.pose = eachCandidate
            plan_response = make_plan(start = start, goal = goal, tolerance = tolerance)
            #Compute the length of the path
            pathLength = self.computePathLength(plan_response.plan)
            #Set the shortestPath for the first candidate
            if firstCandidate:
                shortestPath = pathLength
                self.bestCandidate = eachCandidate
                firstCandidate = False
            #If this is the shortest path until now, we found a new bestCandidate
            if pathLength < shortestLength:
                self.bestCandidate = eachCandidate
                shortestLength = pathLength

class MaxQuantityOfInformationNBVAlgorithm(NextBestViewAlgorithm):
    """NBVAlgorithm based on the criteria of quantity of information"""
    def chooseBestCandidate(self):
        maxQuantityOfInformation = 0.0
        for eachCandidate in self.candidates.values():
            q = self.quantityOfNewInformation(eachCandidate)
            print("quantityOfNewInformation: %f"%q)
            if q > maxQuantityOfInformation:
                maxQuantityOfInformation = q
                self.bestCandidate = eachCandidate

class MCDMPrometheeNBVAlgorithm(NextBestViewAlgorithm):
    """NBVAlgorithm based on PROMETHEE II Multi-criteria decision making method"""
    # name of choosen criteria
    criteria = ['Distance', 'QuantityOfInformation']
    # weights of choosen criteria
    weights = {'Distance': 0.6, 'QuantityOfInformation': 0.4}
    # Preference function used (see paper for details)
    preferenceFunction = {'Distance': PyMCDA.GaussianPreferenceFunction(10), 
                          'QuantiteOfInformation' : PyMCDA.LinearPreferenceFunction(60,10)} 

    def chooseBestCandidate(self):
        #Wait for the availability of this service
        rospy.wait_for_service('move_base/make_plan')
        #Get a proxy to execute the service
        make_plan = rospy.ServiceProxy('move_base/make_plan', GetPlan)

        self.bestCandidate = None
        start = PoseStamped()
        start.header.frame_id = "map"
        start.pose = self.robot_pose
        goal = PoseStamped()
        goal.header.frame_id = "map"
        tolerance = 0.0
        # Evaluation of each candidates for each criteria used
        c = []        #Find the candidate with the shortest path
        for eachCandidate in self.candidates.values():
            #Execute service for each candidates
            goal.pose = eachCandidate
            plan_response = make_plan(start = start, goal = goal, tolerance = tolerance)
            #Compute the length of the path
            distance = self.computePathLength(plan_response.plan)
            # Compute new quantity of information criteria for the candidate
            qi = self.quantityOfNewInformation(eachCandidate)
            # We negated Distance because we want to minimize this criteria
            c.append({'Distance': - distance, 'QuantityOfInformation': qi * 100, 'pose':eachCandidate})
        # Suppresion of candidates that are not on Pareto front
        filteredCandidates = PyMCDA.paretoFilter(c, self.criteria) 
        decision = PyMCDA.decision(filteredCandidates, self.criteria, 
                                   self.weights, self.preferenceFunction)
        if decision:
            self.bestCandidate = decision['pose']

def main(argv):
    if len(argv) < 2:
        sys.stderr.write(__doc__)
        return 1

    #Use the Python reflection API to run the suitable NBV class
    classToLaunch = argv[1]
    print(classToLaunch)
    nbv = getattr(sys.modules[__name__], classToLaunch)()
    nbv.run()
    return 0

if __name__ == "__main__":
    sys.exit(main(sys.argv))
