#######################################################################################################
# Author: An T. Le, DOB: 11/11/1997
# Organization: Vietnamese-German University
# Date Updated: March 21th, 2017
# Date Created: July 2nd, 2016
# Application: Occupancy Grid Library and Search-based Path Planning Library 
#######################################################################################################

from time import sleep


##################################### Concept Robot with Rnage Sensor in Virtual Map ###############################################################
class Robot:
    def __init__(self, sensor_range, speed, grid, startP):
        self.sensor_range = sensor_range
        self.speed = speed
        self.path = None
        self.pos = startP
        self.knownWorld = [] # robot knowledge about its surroundings
        self.isStop = False
        self.pred_of = {}
        
        self.ix = grid.ix
        self.iy = grid.iy

    def traverse(self): # traverse routine 
        if self.pos != self.path[-1] and self.pos in self.path:
            next = self.path[self.path.index(self.pos) + 1]
            if next not in self.knownWorld:
                sleep(1.0 / self.speed)
                self.pos = next
            else:
                self.isStop = True
        else:
            self.isStop = True
    
    def get_data(self, grid): # retrieve data about cells' states 
        result = [(self.pos[0] + i, self.pos[1] + j) for i in range(-self.sensor_range, self.sensor_range + 1) for j in range(-self.sensor_range, self.sensor_range + 1)]
        result.remove(self.pos)
        dark = filter(lambda item: item in grid.walls, result)
        white = filter(lambda item: item not in grid.walls, result)
        return dark, white
    
    def passable(self, id):
        return id not in self.knownWorld
     
    def in_bounds(self, id):
        return 0 <= id[0] and id[0] < self.ix and 0 <= id[1] and id[1] < self.iy
    
    def isPred(self, u, v):
        return self.pred_of[v] == u
    
    def cost(self, currP, nextP):
        if currP in self.knownWorld or nextP in self.knownWorld:
            return float("inf")
        val = 0
        if abs(nextP[1] - currP[1]) + abs(nextP[0] - currP[0]) == 1:
            val += 1
        elif abs(nextP[1] - currP[1]) + abs(nextP[0] - currP[0]) == 2:
            val += 1.414
        return val
    
    def get_pred(self, id):
        x, y = id
        result = [(x + i, y + j) for i in range(-1, 2) for j in range(-1, 2)]
        result.remove(id)
        result = filter(self.in_bounds, result)
        result = filter(self.passable, result)
        result = filter(lambda item: self.isPred(item, id), result)
        if (x+y) % 2 == 0: result.reverse()
        return result
    
    def detect_neighbor(self, id):
        x, y = id
        result = [(x + i, y + j) for i in range(-1, 2) for j in range(-1, 2)]
        result.remove(id)
        result = filter(self.in_bounds, result)
        result = filter(self.passable, result)
        if (x+y) % 2 == 0: result.reverse()
        return result
    
    def detect_changes(self, grid): # detect change in states of robot's surroundings
        sensorData = self.get_data(grid)
        toHigh = filter(lambda item: item not in self.knownWorld, sensorData[0])
        toLow = filter(lambda item: item in self.knownWorld, sensorData[1])
        return toHigh, toLow
    
    def update_cell(self, id): # update a cell in robot knowledge
        if id in self.knownWorld:
            self.knownWorld.remove(id)
        else:
            self.knownWorld.append(id)
    
    def update_map(self, changedNode): # update robot knowledge
        self.knownWorld += changedNode[0]
        for node in changedNode[1]:
            self.knownWorld.remove(node)