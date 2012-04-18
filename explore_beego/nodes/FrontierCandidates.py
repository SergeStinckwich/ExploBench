#!/usr/bin/env python

import roslib
roslib.load_manifest('visualization_msgs')
roslib.load_manifest('nav_msgs')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('sensor_msgs')

import sys
import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from FrontierDetector import FrontierDetector

class FrontierCandidates(object):
    robot_pose = None
    marker_candidates = None
    marker_seq = 0
    occupancy_grid = None

    def __init__(self):
        self._node_name = "FrontierCandidates"
        rospy.init_node(self._node_name)
        rospy.Subscriber('explore/map', OccupancyGrid, self.handle_occupancy_grid)
        rospy.Subscriber('odom', Odometry, self.handle_odom)
        self.marker_candidates = rospy.Publisher('visualization_marker', Marker)
        self.marker_seq = 0

    def handle_odom(self, msg):
            self.robot_pose = msg.pose.pose
            
    def handle_occupancy_grid(self, msg):
        self.occupancy_grid = msg

    def add_marker(self, pose, marker_id):
        marker = Marker()
        self.marker_seq += 1
        marker.id = marker_id
        marker.pose = pose
        marker.text = str(res)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.header.frame_id = "map"
        marker.header.seq = self.marker_seq
        marker.header.stamp = rospy.Time.now()
        marker.action = Marker.ADD
        marker.ns = "marker"
        marker.color.r = 1
        marker.color.a = .5
        marker.scale.z = 1
        marker.lifetime.secs = 30
        self.marker_candidates.publish(marker)

    def run(self):
        while True:
            if self.occupancy_grid == None:
                continue
            data = self.occupancy_grid.data
            width = self.occupancy_grid.info.width
            height = self.occupancy_grid.info.height
            resolution = self.occupancy_grid.info.resolution
            f = FrontierDetector(data, width, height)
            pose1d = int((self.robot_pose.position.x/resolution)*width+(self.robot_pose.position.y/resolution))
            frontiers = f.wavefront_frontier_detector(pose1d)
            i = 0
            for eachFrontier in frontiers:
                i = i + 1
                candidate = f.centroid(eachFrontier)
                self.add_marker(candidate, i)

def main(argv):
    fc = FrontierCandidates()
    fc.run()
    return 0

if __name__ == "__main__":
    sys.exit(main(sys.argv))
