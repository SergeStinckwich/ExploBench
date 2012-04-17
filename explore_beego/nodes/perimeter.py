import unittest
import Queue

def build_frontier(data, width, height):
    """ get the frontier of the known area from the occupancy grid

    :param data: The map data, in row-major order, starting with (0,0).
                 Occupancy probabilities are in the range [0,100].
                 Unknown is -1.
    :param width: grid width
    :param height: grid height
    :returns: list of point [(x1,y1), (x2,y2), ...]
    """
    p = []
    def is_unknown_with_known_neighbour(d,w,h,x,y):
        pos = y*w + x
        pos_above = pos - w # (y-1)*w + x
        pos_below = pos + w # (y+1)*w + x
        return d[pos] == -1 and ( d[pos-1] != -1 or d[pos+1] != -1 or 
          d[pos_above-1] != -1 or d[pos_above] != -1 or d[pos_above+1] != -1 or 
          d[pos_below-1] != -1 or d[pos_below] != -1 or d[pos_below+1] != -1)
    # ignore the main border (1 px)
    for x in xrange(1, width-1):
        for y in xrange(1, height-1):
            pose = (x,y)
            if is_a_frontier_point(data, width, height, pose):
                p.append((x, y))
    return p

def dump(d,w,h):
    import sys
    for y in xrange(h):
        for x in xrange(w):
            v = d[y*w+x]
            if   v == -1: sys.stdout.write('?')
            elif v >=  0: sys.stdout.write('%i'%v if v < 9 else "P")
            elif v == -2: sys.stdout.write('*')
        print('')

def test():
    w = 20
    h = 10
    d = [-1]*w*h
    center = h/2*w+w/2
    d[center] = 5   # (10,5)
    d[center-1] = 5 # (10,4)
    d[center+1] = 5 # (10,6)
    p = build_frontier(d, w, h)
    print(p)
    # debug
    for (x,y) in p:
        d[y*w + x] = -2 # debug value "border"
    dump(d,w,h)

def is_a_frontier_point(data, width, height, pose):
    position = pose[1]*width + pose[0]
    # Return True if unknown and one of the neightbours is known
    if (data[position] == -1):
        pos_above = position - width
        pos_below = position + width
        return ( data[position-1] == 0 or data[position+1] == 0 or 
                 data[pos_above-1] == 0 or data[pos_above] == 0 or data[pos_above+1] == 0 or 
                 data[pos_below-1] == 0 or data[pos_below] == 0 or data[pos_below+1] == 0)
    return False
    
def wavefront_frontier_detector(data, width, height, pose):
    map_open_list = 1
    map_close_list = 2
    
    qm = Queue.Queue()
    qm.put((pose, map_open_list))
    while !(qm.empty()):
        p = qm.get()
        if (p[1] == map_close_list):
            pass
        if is_a_frontier_point(data, width, height, p[0]):
            
    

class FrontierCandidatesTest(unittest.TestCase):
    def test_no_frontier_when_everything_is_unknown(self):
        width = 50
        height = 30
        data = [-1]*width*height
        expected = []
        self.assertEquals(expected, build_frontier(data, width, height))

    def test_no_frontier_when_everything_is_known(self):
        width = 50
        height = 30
        data = [0]*width*height
        expected = []
        self.assertEquals(expected, build_frontier(data, width, height))
    
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
        for each_pose in expected:
            self.assertEquals(True, is_a_frontier_point(data, width, height, each_pose))
                              
    def test_known_square_4x4_in_center_6x6_grid(self):
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
        self.assertEquals(expected, build_frontier(data, width, height))
        
if __name__ == "__main__":
    unittest.main(exit=False)
    test()
