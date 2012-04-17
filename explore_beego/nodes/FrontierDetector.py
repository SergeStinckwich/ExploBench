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
        position = self.position_1d(pose)
        # Return True if unknown and one of the neightbours is known
        if (self.data[position] == -1):
            pos_above = position - self.width
            pos_below = position + self.width
            return ( self.data[position-1] == 0 or self.data[position+1] == 0 or 
                     self.data[pos_above-1] == 0 or self.data[pos_above] == 0 or self.data[pos_above+1] == 0 or 
                     self.data[pos_below-1] == 0 or self.data[pos_below] == 0 or self.data[pos_below+1] == 0)
        return False

    def position_1d(self, pose):
        return (pose[1]*self.width + pose[0])

    def mark(self, pose, value):
        # Mark a pose with a value
        mark[self.position_1d(pose)] = value

    def mark(self,pose):
        return(mark[self.position_1d(pose)])

    def adj(self, pose):
        # return adjacents poses to pose
        # in the linear array
        pos_above = pose - self.width
        pos_below = pose + self.width
        return ([pose-1, pose+1, pos_above-1, pos_above,
                 pos_above+1, pos_below-1, pos_below, pos_below+1])

    def one_of_my_neighbours_is_map_open_space(self, pose):
        map_open_space = 0
        pos_above = position - self.width
        pos_below = position + self.width
        return ( self.data[position-1] == map_open_space or self.data[position+1] == map_open_space or 
                 self.data[pos_above-1] == map_open_space or self.data[pos_above] == map_open_space or self.data[pos_above+1] == map_open_space or 
                 self.data[pos_below-1] == map_open_space or self.data[pos_below] == map_open_space or self.data[pos_below+1] == map_open_space)

    def wavefront_frontier_detector(self, pose):
        """WFD algorithm implementation (see paper Fast Frontier Detection for Robot Exploration)"""
        """Returns a list of frontiers"""
        map_open_space = 0
        map_open_list = 1
        map_close_list = 2
        frontier_open_list = 3
        
        # list of frontiers
        frontiers = []

        self.mark = [map_open_space]*width*height
        
        qm = Queue.Queue()
        qm.put(pose)
        self.mark(pose, map_open_list)
        while (not(qm.empty())):
            p = qm.get()
            if (self.mark(p) == map_close_list):
                pass
            elif self.is_a_frontier_point(p):
                qf = Queue.Queue()
                new_frontier = []
                qf.put(p)
                mark(p, frontier_open_list)
                while (not(qf.empty())):
                    q = qf.get()
                    m = mark(q)
                    if (m == map_close_list) or (m == frontier_close_list):
                        pass
                    elif (q.is_a_frontier_point()):
                        new_frontier.add(q) # add q to the new frontier
                        for each_pose in self.adj(q):
                            m = mark(each_pose)
                            if (m != frontier_open_list) and (m != frontier_close_list) and (m != map_close_list):
                                qf.put(each_pose)
                                mark(each_pose, frontier_open_list)
                    mark(q, frontier_close_list)
                    frontiers.add(new_frontier)
            for each_pose in self.adj(p):
                m = mark(each_pose)
                if (m != map_open_list) and (m != map_close_list) and (self.one_of_my_neighbours_is_map_open_space(each_pose)):
                    qm.put(each_pose)
                    mark(each_pose, map_open_list)
            mark(p, map_close_list)

        return(frontiers)

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

    def test_number_of_adjacent_cells_is_8(self):
        width = 6
        height = 6
        data = [-1]*width*height
        f = FrontierDetector(data, width, height)
        self.assertEquals(8, len(f.adj(9)))
                                      
if __name__ == "__main__":
    unittest.main(exit=False)
