#!/usr/bin/env python

import sys
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('nav_msgs')
roslib.load_manifest('sensor_msgs')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('visualization_msgs')
import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from visualization_msgs.msg import Marker
from FrontierDetector import FrontierDetector

class FrontierCandidates(object):
    def __init__(self):
        self._node_name = "FrontierCandidates"
        rospy.init_node(self._node_name)
        rospy.Subscriber('map', OccupancyGrid, self.handle_occupancy_grid)
        rospy.Subscriber('odom', Odometry, self.handle_odom)
        self.marker_candidates = rospy.Publisher('visualization_marker', Marker)
        self.marker_seq = 0
        self.robot_pose = None
        self.occupancy_grid = None

    def handle_odom(self, msg):
        self.robot_pose = msg.pose.pose

    def handle_occupancy_grid(self, msg):
        self.occupancy_grid = msg

    def add_marker(self, x, y, marker_id):
        marker = Marker()
        self.marker_seq += 1
        marker.id = marker_id
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.text = str(marker_id)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.header.frame_id = "map"
        marker.header.seq = self.marker_seq
        marker.header.stamp = rospy.Time.now()
        marker.action = Marker.ADD
        marker.ns = "frontier2"
        marker.color.r = 1
        marker.color.a = .5
        marker.scale.z = 1
        marker.lifetime.secs = 30
        self.marker_candidates.publish(marker)

    def run(self):
        #rospy.wait_for_service('dynamic_map')
        #get_map = rospy.ServiceProxy('dynamic_map', GetMap)
        while not rospy.is_shutdown():
            import pdb; pdb.set_trace()
            if not self.robot_pose:
                rospy.sleep(0.5)
                continue
            if not self.occupancy_grid:
                # FIXME resp = get_map()
                #self.occupancy_grid = resp.response.map
                rospy.sleep(1.0)
                continue
            data = self.occupancy_grid.data
            width = self.occupancy_grid.info.width
            height = self.occupancy_grid.info.height
            resolution = self.occupancy_grid.info.resolution
            origin_position = self.occupancy_grid.info.origin.position
            f = FrontierDetector(data, width, height)
            # TODO pose 0,0 center -> 0,0 up-left corner
            pose1d = int(width * (self.robot_pose.position.x - origin_position.x) / resolution +
                        (self.robot_pose.position.y - origin_position.y) / resolution)
            frontiers = f.wavefront_frontier_detector(pose1d)
            def xy_from_pose1d(p):
                x = origin_position.x + (p % width) * resolution
                y = origin_position.y + (p // width) * resolution
                return (x, y)
            # FIXME (x, y) = xy_from_pose1d(pose1d)
            # self.add_marker(x, y, 0)
            if not frontiers:
                print("no frontier at %s"%str(xy_from_pose1d(pose1d)))
            i = 0
            for eachFrontier in frontiers:
                i += 1
                candidate = f.centroid(eachFrontier)
                (x, y) = xy_from_pose1d(candidate)
                self.add_marker(x, y, i)
            rospy.sleep(0.5)

def main(argv):
    fc = FrontierCandidates()
    fc.run()
    return 0

if __name__ == "__main__":
    sys.exit(main(sys.argv))
