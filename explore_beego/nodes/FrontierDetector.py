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
        # Return True if unknown and one of the neightbours is known
        if (self.data[pose] == -1):
            pos_above = pose - self.width
            pos_below = pose + self.width
            adj = self. adj(pose)
            for each_pose in adj:
                if self.data[each_pose] == 0:
                    return True
        return False

    def adj(self, pose):
        # return adjacents poses to pose
        # in the linear array
        pos_above = pose - self.width
        pos_below = pose + self.width        
        # corner left top
        if (pose == 0):
            return([                  pose + 1,
                    pose+ self.width, pose + 1 + self.width])
        # corner right top
        if (pose == self.width - 1):
            return([self.width - 2,
                    2 * self.width - 2,
                    2 * self.width - 1])
        # corner left down
        if (pose == self.width * self.height - self.width):
            return([self.width * self.height - 2 * self.width,
                    self.width * self.height - 2 * self.width + 1,
                    self.width * self.height - self.width + 1])
        # corner right down
        if (pose == self.width * self.height - 1):
            return([self.width * self.height - self.width - 2,
                    self.width * self.height - self.width - 1,
                    self.width * self.height - 2])
        # border top
        if (pose < self.width):
            return([pose - 1,                                 pose + 1,
                    pose - 1 + self.width, pose + self.width, pose + 1 + self.width])
        # border down
        if (pose >= self.width * self.height - self.width - 1) and (pose < self.width * self.height):
            return([pose - 1 - self.width, pose - self.width, pose + 1 - self.width,
                    pose - 1                                , pose + 1])
        # border right
        if ((pose+1) % self.width == 0):
            return([pose - 1 - self.width, pose - self.width,
                    pose - 1,
                    pose - 1 + self.width, pose + self.width])
        # border left
        if (pose % self. width == 0):
            return([pose - self.width, pose + 1 - self.width,
                                       pose + 1,
                    pose + self.width, pose + 1 + self.width])
        # else
        return ([pos_above - 1  , pos_above, pos_above + 1,
                 pose - 1                  , pose + 1, 
                 pos_below - 1  , pos_below, pos_below + 1])

    def one_of_my_neighbours_is_map_open_space(self, pose):
        map_open_space = 0
        pos_above = pose - self.width
        pos_below = pose + self.width
        adj = self.adj(pose)
        for each_pose in adj:
            if self.data[each_pose] == map_open_space:
                return True
        return False

    def wavefront_frontier_detector(self, pose):
        """WFD algorithm implementation (see paper Fast Frontier Detection for Robot Exploration)"""
        """Returns a list of frontiers"""
        map_open_space = 0
        map_open_list = 1
        map_close_list = 2
        frontier_open_list = 3
        frontier_close_list = 4

        # list of frontiers
        frontiers = []

        self.mark = [map_open_space]*self.width*self.height
        
        qm = Queue.Queue()
        qm.put(pose)
        self.mark[pose] = map_open_list
        while (not(qm.empty())):
            p = qm.get()
            if (self.mark[p] == map_close_list):
                pass
            elif self.is_a_frontier_point(p):
                qf = Queue.Queue()
                new_frontier = []
                qf.put(p)
                self.mark[p] = frontier_open_list
                while (not(qf.empty())):
                    q = qf.get()
                    m = self.mark[q]
                    if (m == map_close_list) or (m == frontier_close_list):
                        pass
                    elif (self.is_a_frontier_point(q)):
                        new_frontier.append(q) # add q to the new frontier
                        for each_pose in self.adj(q):
                            m = self.mark[each_pose]
                            if (m != frontier_open_list) and (m != frontier_close_list) and (m != map_close_list):
                                qf.put(each_pose)
                                self.mark[each_pose] = frontier_open_list
                    self.mark[q] = frontier_close_list
                    if not(new_frontier in frontiers):
                        frontiers.append(new_frontier)
            for each_pose in self.adj(p):
                m = self.mark[each_pose]
                if (m != map_open_list) and (m != map_close_list) and (self.one_of_my_neighbours_is_map_open_space(each_pose)):
                    qm.put(each_pose)
                    self.mark[each_pose] = map_open_list
            self.mark[p] = map_close_list

        return(frontiers)

class FrontierDetectorTest(unittest.TestCase):
    
    def test_cell_on_the_corner_left_top(self):
        width = 6
        height = 5
        data = [-1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1,
                -1, -1,  0,  0, -1, -1,
                -1, -1,  0,  0, -1, -1,
                -1, -1, -1, -1, -1, -1]
        f = FrontierDetector(data, width, height)
        expected = [   1,
                    6, 7]
        self.assertEquals(expected, f.adj(0))
        
    def test_cell_on_the_corner_right_top(self):
        width = 6
        height = 5
        data = [-1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1,
                -1, -1,  0,  0, -1, -1,
                -1, -1,  0,  0, -1, -1,
                -1, -1, -1, -1, -1, -1]
        f = FrontierDetector(data, width, height)
        expected = [4,
                    10, 11]
        self.assertEquals(expected, f.adj(5))

    def test_cell_on_the_corner_left_down(self):
        width = 6
        height = 5
        data = [-1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1,
                -1, -1,  0,  0, -1, -1,
                -1, -1,  0,  0, -1, -1,
                -1, -1, -1, -1, -1, -1]
        f = FrontierDetector(data, width, height)
        expected = [18, 19, 
                        25]
        self.assertEquals(expected, f.adj(24))

    def test_cell_on_the_border_top(self):
        width = 6
        height = 5
        data = [-1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1,
                -1, -1,  0,  0, -1, -1,
                -1, -1,  0,  0, -1, -1,
                -1, -1, -1, -1, -1, -1]
        f = FrontierDetector(data, width, height)
        expected = [1,    3, 
                    7, 8, 9]
        self.assertEquals(expected, f.adj(2))

    def test_cell_on_the_border_down(self):
        width = 6
        height = 5
        data = [-1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1,
                -1, -1,  0,  0, -1, -1,
                -1, -1,  0,  0, -1, -1,
                -1, -1, -1, -1, -1, -1]
        f = FrontierDetector(data, width, height)
        expected = [20, 21, 22,
                    26,     28]
        self.assertEquals(expected, f.adj(27))

    def test_cell_on_the_border_right(self):
        width = 6
        height = 5
        data = [-1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1,
                -1, -1,  0,  0, -1, -1,
                -1, -1,  0,  0, -1, -1,
                -1, -1, -1, -1, -1, -1]
        f = FrontierDetector(data, width, height)
        expected = [10, 11,
                    16,  
                    22, 23]
        self.assertEquals(expected, f.adj(17))

    def test_cell_on_the_border_left(self):
        width = 6
        height = 5
        data = [-1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1,
                -1, -1,  0,  0, -1, -1,
                -1, -1,  0,  0, -1, -1,
                -1, -1, -1, -1, -1, -1]
        f = FrontierDetector(data, width, height)
        expected = [6 ,  7,
                        13, 
                    18, 19]
        self.assertEquals(expected, f.adj(12))

    def test_no_frontier_when_everything_is_unknown(self):
        width = 6
        height = 5
        data = [-1]*width*height
        expected = []
        f = FrontierDetector(data, width, height)
        for each_pose in range(0, width*height-1):
                self.assertEquals(False, f.is_a_frontier_point(each_pose))

    def test_cell00_has_3_adjacent_cells(self):
        width = 6
        height = 6
        data = [-1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1,
                -1, -1,  0,  0, -1, -1,
                -1, -1,  0,  0, -1, -1,
                -1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1]
        f = FrontierDetector(data, width, height)
        self.assertEquals(3, len(f.adj(0)))

    def test_unknown_env_has_no_frontiers(self):
        width = 6
        height = 6
        data = [-1]*width*height
        f = FrontierDetector(data, width, height)
        pose = 15
        self.assertEquals([], f.wavefront_frontier_detector(pose))

    def test_known_square_4x4_in_center_6x6_grid_are_frontiers_points(self):
        width = 6
        height = 6
        data = [-1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1,
                -1, -1,  0,  0, -1, -1,
                -1, -1,  0,  0, -1, -1,
                -1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1]
        expected = [7 , 8, 9,10,
                    13,      16,
                    19,      22,
                    25,26,27,28]
        f = FrontierDetector(data, width, height)
        for each_pose in expected:
            self.assertEquals(True, f.is_a_frontier_point(each_pose))
        print f.wavefront_frontier_detector(12)
        self.assertEquals(expected, (f.wavefront_frontier_detector(12)))
        
    def test_number_of_adjacent_cells_is_8(self):
        width = 6
        height = 6
        data = [-1]*width*height
        f = FrontierDetector(data, width, height)
        self.assertEquals(8, len(f.adj(9)))
         
if __name__ == "__main__":
    unittest.main(exit=False)
