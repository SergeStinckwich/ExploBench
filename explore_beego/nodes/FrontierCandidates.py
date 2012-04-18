import FrontierDetector
import roslib
roslib.load_manifest('visualization_msgs')
roslib.load_manifest('nav_msgs')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('sensor_msgs')
import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker

class FrontierCandidates(object):
    robot_pose = None

    def __init__(self):
        self._node_name = "FrontierCandidates"
        rospy.init_node(self._node_name)
        rospy.Subscriber('visualization_marker', Marker, self.handle_markers)
        rospy.Subscriber('explore/map', OccupancyGrid, self.handle_occupancy_grid)
        rospy.Subscriber('odom', Odometry, self.handle_odom)
        self.marker_dbg = rospy.Publisher('visualization_marker', Marker)
        self.marker_dbg_seq = 0

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
        self.publisher.publish(marker)

    def run(self):
        while True:
            data = self.occupancy_grid.data
            width = self.occupancy_grid.info.width
            height = self.occupancy_grid.info.height
            f = FrontierDetector(data, width, height)
            frontiers = f.wavefront_frontier_detector(robot_pose)
            i = 0
            for eachFrontier in frontiers:
                i = i + 1
                candidate = f.centroid(eachFrontier)
                self.add_marker(candidate, i)

    def main(self):
        run()
        return 0

if __name__ == "__main__":
    sys.exit(main(sys.argv))
