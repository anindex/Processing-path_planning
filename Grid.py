class InvalidResolution(RuntimeError):
    def __init__(self, arg):
        self.arg = arg

def draw_cell(x, y, res, attr):
        stroke(0)
        fill(attr[0], attr[1], attr[2])
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
        if dir == 'd':
            x1, y1 = x + res / 2, y + res / 4
            x2, y2 = x + res / 2, y + 3 * res /4
            line(x1, y1, x2, y2)
            pushMatrix()
            translate(x2, y2)
            a = atan2(x1-x2, y2-y1)
            rotate(a)
            line(0, 0, -2, -2)
            line(0, 0, 2, -2)
            popMatrix()
        if dir == 'l':
            x1, y1 = x + 3 * res / 4, y + res/ 2
            x2, y2 = x + res / 4, y + res /2
            line(x1, y1, x2, y2)
            pushMatrix()
            translate(x2, y2)
            a = atan2(x1-x2, y2-y1)
            rotate(a)
            line(0, 0, -2, -2)
            line(0, 0, 2, -2)
            popMatrix()
        if dir == 'r':
            x1, y1 = x + res / 4, y + res/ 2
            x2, y2 = x + 3 * res / 4, y + res /2
            line(x1, y1, x2, y2)
            pushMatrix()
            translate(x2, y2)
            a = atan2(x1-x2, y2-y1)
            rotate(a)
            line(0, 0, -2, -2)
            line(0, 0, 2, -2)
            popMatrix()

class SquareGrid:
    DIRS = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
    
    def __init__(self, w, h, resolution):
        self.resolution = resolution
        self.f = createFont("Arial", resolution / 2, True)
        self.walls = []
        self.roughs = []
        if(w % resolution == 0 and h % resolution == 0):
            self.ix = w / resolution
            self.iy = h / resolution
        else:
            raise InvalidResolution("Bad Initialization")
        
    def in_bounds(self, id):
        return 0 <= id[0] and id[0] < ix and 0 <= id[1] and id[1] < iy
    
    def passable(self, id):
        return id not in self.walls
     
    def neighbor(self, id):
        x, y = id
        #result = [(x+i, y+j) for i in range(-1:2) for j in range(-1:2)]
        result = [ (x+1, y), (x+1, y+1), (x, y+1), (x-1, y+1), (x-1, y), (x-1, y-1), (x, y-1), (x+1, y-1) ]
        result = filter(self.in_bounds and self.passable, result)
        if (x+y) % 2 == 0: result.reverse()
        return result
        
    
    def add_walls(self, lp, rp):
        for i in range(lp[0], rp[0]+1):
            for j in range(lp[1], rp[1]+1):
                self.walls.append((i, j))
    
    def cost(self, currP, nextP):
        val = 1
        if abs(nextP[1] - currP[1]) + abs(nextP[0] - currP[0]) == 2:
            val += 0.414
        if nextP in self.roughs:
            val += 4
        return val 
    
    def draw_grid(self, startP, goal, distances = None, came_from = None, path = None):
        for row in range(self.iy):
            for col in range(self.ix):
                current = (col, row)
                if current in self.walls:
                    draw_cell(col, row, self.resolution, (0, 0, 0))
                elif current in self.roughs:
                    draw_cell(col, row, self.resolution, (100, 100, 100))
                elif distances != None and current in distances:
                    draw_cell(col, row, self.resolution, (255, 255, 255))
                    text(self.f, distances[current], col*self.resolution + self.resolution / 2, row*self.resolution + self.resolution/ 2)
                elif came_from != None and current in came_from:
                    draw_cell(col, row, self.resolution, (255, 255, 255))
                    x, y = current
                    px, py = came_from[current]
                    if px == x - 1:
                        draw_arrow(col, row, self.resolution, 'l')
                    elif px == x + 1:
                        draw_arrow(col, row, self.resolution, 'r')
                    elif py == y - 1:
                        draw_arrow(col, row, self.resolution, 'u')
                    elif py == y + 1:
                        draw_arrow(col, row, self.resolution, 'd')
                elif path != None and current in path:
                    draw_cell(col, row, self.resolution, (255, 0, 0))
                elif current == startP or current == goal:
                    draw_cell(col, row, self.resolution, (0, 255, 0))
                else:
                    draw_cell(col, row, self.resolution, (255, 255, 255))
            