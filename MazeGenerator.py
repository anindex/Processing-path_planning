from random import randint
from Grid import SquareGrid
from time import sleep

class MazeGenerator:
    def __init__(self, w, h, res, initial_cell):
        self.grid = SquareGrid(w, h, res)
        self.grid.walls = [(x, y) for x in range(self.grid.ix) for y in range(self.grid.iy) if x % 2 == 0 or y % 2 == 0]
        if initial_cell[0] % 2 == 0 or initial_cell[1] % 2 == 0:
            self.fcell = (1, 1)
            print "Cell coordinates must be odd, initial cell sets to (1, 1)"
        else:
            self.fcell = initial_cell        
    
    def in_bounds(self, id):
        return 0 <= id[0] and id[0] < self.grid.ix and 0 <= id[1] and id[1] < self.grid.iy
    
    def neighbors(self, id):
        neighbor = [(id[0], id[1] - 2), (id[0] + 2, id[1]), (id[0], id[1] + 2), (id[0] - 2, id[1])] #<-------------------- clockwise
        neighbor = filter(self.in_bounds, neighbor)
        return neighbor
    
    
    def generate(self):
        #-------------------- Initialize --------------------
        stack = []
        unvisited = [(x, y) for x in range(1, self.grid.ix, 2) for y in range(1, self.grid.iy, 2)]
        
        stack.append(self.fcell)
        unvisited.remove(self.fcell)
        current = self.fcell
        
        # ------------------ Main loop ----------------------
        while unvisited:
            movements = [x for x in self.neighbors(current) if x in unvisited]

            if movements:
                next_move = movements[randint(0, len(movements) - 1)]
                stack.append(next_move)
                wall = (current[0] + (next_move[0] - current[0]) / 2, current[1] + (next_move[1] - current[1]) / 2)                
                self.grid.walls.remove(wall)
                
                unvisited.remove(next_move)
                current = next_move
            else:
                current = stack.pop()

    
            
            
            
            
            
        
        