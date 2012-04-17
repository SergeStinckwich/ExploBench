import unittest
import Queue

class FrontierDetector(object):
    
    data = []
    mark = []
    

    def __init__(self, data, width, height):
        self.data = data
        self.width = width
        self.height = height

    def is_a_frontier_point(self, pose):
        position = pose[1]*self.width + pose[0]
        # Return True if unknown and one of the neightbours is known
        if (self.data[position] == -1):
            pos_above = position - self.width
            pos_below = position + self.width
            return ( self.data[position-1] == 0 or self.data[position+1] == 0 or 
                     self.data[pos_above-1] == 0 or self.data[pos_above] == 0 or self.data[pos_above+1] == 0 or 
                     self.data[pos_below-1] == 0 or self.data[pos_below] == 0 or self.data[pos_below+1] == 0)
        return False

    def position_1d(pose):
        return 0

    def wavefront_frontier_detector(data, width, height, pose):
        map_open_list = 1
        map_close_list = 2
        self.mark = [0]*width*height
        
        qm = Queue.Queue()
        qm.put(pose)
        mark[position_1d(pose)] = map_open_list
        mark[pose] = map_open_list
        while (not(qm.empty())):
            p = qm.get()
            if (p[1] == map_close_list):
                pass
            if is_a_frontier_point(data, width, height, p[0]):
                qf = Queue.Queue()
                new_frontier = []
            

class FrontierDetectorTest(unittest.TestCase):
    def test_no_frontier_when_everything_is_unknown(self):
        width = 50
        height = 30
        data = [-1]*width*height
        expected = []
        f = FrontierDetector(data, width, height)
        for i in range(0, width-1):
            for j in range(0, height-1):
                self.assertEquals(False, f.is_a_frontier_point([i,j]))

    def test_known_square_4x4_in_center_6x6_grid_are_frontiers_points(self):
        width = 6
        height = 6
        data = [-1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1,
                -1, -1,  0,  0, -1, -1,
                -1, -1,  0,  0, -1, -1,
                -1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1]
        expected = [(1,1), (1,2), (1,3), (1,4),
                    (2,1),               (2,4),
                    (3,1),               (3,4),
                    (4,1), (4,2), (4,3), (4,4)]
        f = FrontierDetector(data, width, height)
        for each_pose in expected:
            self.assertEquals(True, f.is_a_frontier_point(each_pose))
                                      
if __name__ == "__main__":
    unittest.main(exit=False)
