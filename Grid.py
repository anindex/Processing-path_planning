from heapq import heappush, heappop, heapify

class PriorityDict(dict):
    def __init__(self, *args, **kwargs):
        super(PriorityDict, self).__init__(self, *args, **kwargs)
        self._rebuild_heap()
    
    def _rebuild_heap(self):
        self._heap = [(v, k) for k, v in self.iteritems()]
        heapify(self._heap)        
    
    def pop(self):
        v, k = heappop(self._heap)
        while k not in self or self[k] != v:
            v, k = heappop(self._heap)
        del self[k]
        return k, v
    
    def top(self):
        heap  = self._heap
        v, k = heap[0]
        while k not in self or self[k] != v:
            heappop(heap)
            v, k = heap[0]
        return k, v
            
    def __setitem__(self, k, v):
        super(PriorityDict, self).__setitem__(k, v)
        
        if len(self._heap) < 2 * len(self):
            heappush(self._heap, (v, k))
        else:
            self._rebuild_heap()


class InvalidResolution(RuntimeError):
    def __init__(self, arg):
        self.arg = arg

def draw_cell(x, y, res, attr):
        stroke(0)
        fill(attr[0], attr[1], attr[2])
        rect(x*res, y*res, res, res)
    
def draw_cell_value(x, y, res, value):
        stroke(0)
        fill(value)
        rect(x*res, y*res, res, res)

def draw_arrow(x, y, res, dir):
        if dir == 'u':
            x1 = x*res + res / 2
            y1 = y*res + 3*res/4
            x2 = x*res + res / 2
            y2 = y*res + res /4
            line(x1, y1, x2, y2)
            pushMatrix()
            translate(x2, y2)
            a = atan2(x1-x2, y2-y1)
            rotate(a)
            line(0, 0, -2, -2)
            line(0, 0, 2, -2)
            popMatrix()
        elif dir == 'd':
            x1, y1 = x*res + res / 2, y*res + res / 4
            x2, y2 = x*res + res / 2, y*res + 3 * res /4
            line(x1, y1, x2, y2)
            pushMatrix()
            translate(x2, y2)
            a = atan2(x1-x2, y2-y1)
            rotate(a)
            line(0, 0, -2, -2)
            line(0, 0, 2, -2)
            popMatrix()
        elif dir == 'l':
            x1, y1 = x*res + 3 * res / 4, y*res + res/ 2
            x2, y2 = x*res + res / 4, y*res + res /2
            line(x1, y1, x2, y2)
            pushMatrix()
            translate(x2, y2)
            a = atan2(x1-x2, y2-y1)
            rotate(a)
            line(0, 0, -2, -2)
            line(0, 0, 2, -2)
            popMatrix()
        elif dir == 'r':
            x1, y1 = x*res + res / 4, y*res + res/ 2
            x2, y2 = x*res + 3 * res / 4, y*res + res /2
            line(x1, y1, x2, y2)
            pushMatrix()
            translate(x2, y2)
            a = atan2(x1-x2, y2-y1)
            rotate(a)
            line(0, 0, -2, -2)
            line(0, 0, 2, -2)
            popMatrix()
        elif dir == 'q':
            x1 = x*res + 3 * res / 4
            y1 = y*res + 3*res/4
            x2 = x*res + res / 4
            y2 = y*res + res /4
            line(x1, y1, x2, y2)
            pushMatrix()
            translate(x2, y2)
            a = atan2(x1-x2, y2-y1)
            rotate(a)
            line(0, 0, -2, -2)
            line(0, 0, 2, -2)
            popMatrix()
        elif dir == 'e':
            x1, y1 = x*res + res / 4, y*res + 3 * res / 4
            x2, y2 = x*res + 3 * res / 4, y*res + res /4
            line(x1, y1, x2, y2)
            pushMatrix()
            translate(x2, y2)
            a = atan2(x1-x2, y2-y1)
            rotate(a)
            line(0, 0, -2, -2)
            line(0, 0, 2, -2)
            popMatrix()
        elif dir == 'z':
            x1, y1 = x*res + 3 * res / 4, y*res + res/ 4
            x2, y2 = x*res + res / 4, y*res + 3* res /4
            line(x1, y1, x2, y2)
            pushMatrix()
            translate(x2, y2)
            a = atan2(x1-x2, y2-y1)
            rotate(a)
            line(0, 0, -2, -2)
            line(0, 0, 2, -2)
            popMatrix()
        elif dir == 'c':
            x1, y1 = x*res + res / 4, y*res + res/ 4
            x2, y2 = x*res + 3 * res / 4, y*res + 3 * res /4
            line(x1, y1, x2, y2)
            pushMatrix()
            translate(x2, y2)
            a = atan2(x1-x2, y2-y1)
            rotate(a)
            line(0, 0, -2, -2)
            line(0, 0, 2, -2)
            popMatrix()

class SquareGrid:
    def __init__(self, w, h, resolution):
        self.resolution = resolution
        self.f = createFont("Arial", resolution / 2, True)
        self.walls = []
        if(w % resolution == 0 and h % resolution == 0):
            self.ix = w / resolution
            self.iy = h / resolution
        else:
            raise InvalidResolution("Bad Initialization")
        
    def in_bounds(self, id):
        return 0 <= id[0] and id[0] < self.ix and 0 <= id[1] and id[1] < self.iy
    
    def passable(self, id):
        return id not in self.walls
     
    def neighbor(self, id):
        x, y = id
        result = [(x + i, y + j) for i in range(-1, 2) for j in range(-1, 2)]
        result.remove(id)
        result = filter(self.in_bounds, result)
        result = filter(self.passable, result)
        if (x+y) % 2 == 0: result.reverse()
        return result
    
    def all_neighbor(self, id):
        x, y = id
        result = [(x + i, y + j) for i in range(-1, 2) for j in range(-1, 2)]
        result = filter(self.in_bounds, result)
        if (x+y) % 2 == 0: result.reverse()
        return result
    
    def add_walls(self, lp, rp):
        for i in range(lp[0], rp[0]+1):
            for j in range(lp[1], rp[1]+1):
                self.walls.append((i, j))
    
    def cost(self, currP, nextP):
        val = 0
        if currP in self.knownWorld or nextP in self.knownWorld:
            return float("inf")
        if abs(nextP[1] - currP[1]) + abs(nextP[0] - currP[0]) == 1:
            val += 1
        elif abs(nextP[1] - currP[1]) + abs(nextP[0] - currP[0]) == 2:
            val += 1.414
        if nextP in self.roughs:
            val += 4
        return val 
    def draw_grid(self, startP, goal, bot, distances = None, came_from = None, frontier = None, path = None, closedNode = None):
        for row in range(self.iy):
            for col in range(self.ix):
                current = (col, row)
                if current in self.walls:
                    draw_cell(col, row, self.resolution, (0, 0, 0))
                elif current == bot.pos:
                    draw_cell(col, row, self.resolution, (0, 0, 255))
                elif path != None and current in path:
                    draw_cell(col, row, self.resolution, (255, 0, 0))
                elif closedNode != None and current in closedNode:
                    num = closedNode.count(current)
                    draw_cell_value(col, row, self.resolution, 255 - 60*num)
                elif frontier != None and current in frontier:
                    draw_cell(col, row, self.resolution, (255, 255, 0))
                elif distances != None and current in distances:
                    draw_cell(col, row, self.resolution, (255, 255, 255))
                    text(self.f, distances[current], col*self.resolution + self.resolution / 2, row*self.resolution + self.resolution/ 2)
                elif came_from != None and current in came_from:
                    draw_cell(col, row, self.resolution, (255, 255, 255))
                    x, y = current
                    px, py = came_from[current]
                    if px == x - 1 and py == y:
                        draw_arrow(col, row, self.resolution, 'l')
                    elif px == x + 1 and py == y:
                        draw_arrow(col, row, self.resolution, 'r')
                    elif py == y - 1 and px == x:
                        draw_arrow(col, row, self.resolution, 'u')
                    elif py == y + 1 and px == x:
                        draw_arrow(col, row, self.resolution, 'd')
                    elif px == x - 1 and py == y - 1:
                        draw_arrow(col, row, self.resolution, 'q')
                    elif px == x + 1 and py == y + 1:
                        draw_arrow(col, row, self.resolution, 'c')
                    elif py == y - 1 and px == x + 1:
                        draw_arrow(col, row, self.resolution, 'e')
                    elif py == y + 1 and px == x - 1:
                        draw_arrow(col, row, self.resolution, 'z')
                elif current == startP or current == goal:
                    draw_cell(col, row, self.resolution, (0, 255, 0))
                else:
                    draw_cell(col, row, self.resolution, (255, 255, 255))


def map1(res):
    grid = SquareGrid(width, height, res)
    grid.add_walls((8, 20), (14, 20))
    grid.add_walls((8, 18), (12, 18))
    grid.walls.append((8, 19))
    grid.add_walls((12, 12), (12, 17))
    grid.add_walls((14, 18), (14, 19))
    grid.add_walls((14, 12), (14, 14))
    grid.add_walls((8, 20), (14, 20))
    grid.add_walls((14, 15), (17, 15))
    grid.add_walls((14, 17), (17, 17))
    return grid

def maze1(res):
    grid = SquareGrid(width, height, res)
    grid.add_walls((0,0), (0, 19))
    grid.add_walls((18,0), (18, 19))
    grid.add_walls((1,19), (17, 19))
    grid.add_walls((3,0), (17, 0))
    grid.walls.append((1, 0))
    grid.walls.append((2, 2))
    grid.walls.append((3, 2))
    grid.walls.append((3, 1))
    grid.walls.append((7, 1))
    grid.walls.append((7, 2))
    grid.walls.append((9, 2))
    grid.add_walls((10,2), (10, 5))
    grid.add_walls((12,1), (12, 4))
    grid.add_walls((14,2), (16, 2))
    grid.add_walls((1,4), (8, 4))
    grid.walls.append((8, 5))
    grid.add_walls((8,6), (13, 6))
    grid.add_walls((14,3), (14, 8))
    grid.add_walls((16,4), (17, 4))
    grid.add_walls((16,6), (16, 8))
    grid.walls.append((15, 8))
    grid.walls.append((6, 7))
    grid.add_walls((2,6), (6, 6))
    grid.add_walls((2,7), (2, 8))
    grid.add_walls((3,8), (4, 8))
    grid.add_walls((4,9), (8, 9))
    grid.add_walls((8,8), (12, 8))
    grid.walls.append((10, 9))
    grid.walls.append((2, 10))
    grid.add_walls((2,11), (5, 11))
    grid.add_walls((7,10), (7, 13))
    grid.walls.append((6, 13))
    grid.add_walls((8,11), (12, 11))
    grid.add_walls((12,10), (16, 10))
    grid.walls.append((17, 13))
    grid.add_walls((1,13), (2, 13))
    grid.add_walls((4,12), (4, 14))
    grid.add_walls((2,15), (2, 19))
    grid.add_walls((3,15), (9, 15))
    grid.add_walls((9,13), (9, 14))
    grid.add_walls((4,17), (11, 17))
    grid.add_walls((12,15), (12, 17))
    grid.add_walls((11,12), (11, 15))
    grid.add_walls((14,12), (14, 17))
    grid.add_walls((15,15), (16, 15))
    grid.add_walls((15,17), (17, 17))
    grid.add_walls((15,17), (17, 17))
    grid.add_walls((16,11), (16, 13))
    grid.walls.append((13, 13))
    grid.walls.remove((18, 15))
    return grid