import unittest

def known_perimeter(data, width, height):
    """ get the perimeter of the known area from the occupancy grid

    :param data: The map data, in row-major order, starting with (0,0).
                 Occupancy probabilities are in the range [0,100].
                 Unknown is -1.
    :param width: grid width
    :param height: grid height
    :returns: list of point [(x1,y1), (x2,y2), ...]
    """
    p = []
    def is_unknown_with_known_neighbour(d,w,x,y):
        pos = y*w + x
        pos_above = pos - w # (y-1)*w + x
        pos_below = pos + w # (y+1)*w + x
        return d[pos] == -1 and ( d[pos-1] != -1 or d[pos+1] != -1 or 
          d[pos_above-1] != -1 or d[pos_above] != -1 or d[pos_above+1] != -1 or 
          d[pos_below-1] != -1 or d[pos_below] != -1 or d[pos_below+1] != -1)
    # ignore the main border (1 px)
    for x in range(1, width-1):
        for y in range(1, height-1):
            if is_unknown_with_known_neighbour(data, width, x, y):
                p.append((x, y))
    return p

def dump(d,w,h):
    import sys
    for y in range(h):
        for x in range(w):
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
    p = known_perimeter(d, w, h)
    print(p)
    # debug
    for (x,y) in p:
        d[y*w + x] = -2 # debug value "border"
    dump(d,w,h)

class FrontierCandidates(unittest.TestCase):
    def test_no_frontier_when_everything_is_unknown(self):
        width = 50
        height = 30
        data = [-1]*width*height
        expected = []
        self.assertEquals(expected, known_perimeter(data, width, height))

    def test_no_frontier_when_everything_is_known(self):
        width = 50
        height = 30
        data = [0]*width*height
        expected = []
        self.assertEquals(expected, known_perimeter(data, width, height))

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
        self.assertEquals(expected, known_perimeter(data, width, height))

if __name__ == "__main__":
    unittest.main(exit=False)
    test()
