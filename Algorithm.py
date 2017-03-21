#######################################################################################################
# Author: An T. Le, DOB: 11/11/1997
# Organization: Vietnamese-German University
# Date Updated: March 21th, 2017
# Date Created: July 2nd, 2016
# Application: Occupancy Grid Library and Search-based Path Planning Library 
#######################################################################################################

from Queue import PriorityQueue
from Grid import PriorityDict
from time import sleep

##########################################################
#            A* Search-based algorithm (from start)      #
##########################################################

def astar_search(grid, startP, goal):
    frontier = PriorityQueue()
    came_from = {}
    cost_so_far = {}
    
    cost_so_far[startP] = 0
    came_from[startP] = startP
    frontier.put((heuristic_distance(startP, goal) + cost_so_far[startP], startP))
    
    current = startP
    while not frontier.empty() and current != goal:
        current = frontier.get()[1]
        
        for next in grid.neighbor(current):
            new_cost = cost_so_far[current] + grid.cost(current, next)
            if next not in came_from or new_cost < cost_so_far[next]:
                came_from[next] = current
                cost_so_far[next] = new_cost
                frontier.put((heuristic_distance(next, goal) + cost_so_far[next], next))
        
    return came_from, cost_so_far if current == goal else None

###########################################################
#         D* Lite Search-based algorithm (from goal)      #
###########################################################

#Note: data[node][0] -> g() value
#      data[node][1] -> rhs() value

def calculate_key(node, data, robotPos, km): 
    return min(data[node][0], data[node][1]) + heuristic_distance(robotPos, node) + km

def update_vertex(node, robot, goal, frontier, data, km):
    if node != goal: data[node][1] = get_lowest_cost_node(robot, node, data)[1]
    if node in frontier: del frontier[node]
    if  data[node][0] != data[node][1]: frontier[node] = calculate_key(node, data, robot.pos, km)
        
def get_lowest_cost_node(robot, node, data):
    neighbors = {k: (data[k][0]+robot.cost(node, k)) for k in robot.detect_neighbor(node)}
    result = min(neighbors, key=neighbors.get)
    return result, neighbors[result]

def get_lowest_rhs_node(robot, node, data):
    neighbors = {k: (data[k][1]+robot.cost(node, k)) for k in robot.detect_neighbor(node)}
    result = min(neighbors, key=neighbors.get)
    return result, neighbors[result]

def compute_shortest_path(robot, goal, data, frontier, km, closedNode, Pnode): 
    while frontier and (frontier.top()[1] < (calculate_key(robot.pos, data, robot.pos, km) + 0.2) or data[robot.pos][1] != data[robot.pos][0]):
        currNode, kold = frontier.pop() 
        knew = calculate_key(currNode, data, robot.pos, km)
        if kold < knew:
            frontier[currNode] = knew
        elif data[currNode][0] > data[currNode][1]:
            data[currNode][0] = data[currNode][1]
            for pred in robot.detect_neighbor(currNode):
                update_vertex(pred, robot, goal, frontier, data, km)
        else:
            data[currNode][0] = float("inf")
            for node in robot.detect_neighbor(currNode) + [currNode, ]:
                update_vertex(node, robot, goal, frontier, data, km)
        closedNode.append(currNode)
        Pnode += 1
        #sleep(0.1)
    return Pnode


def dlite_search(grid, robot, startP, goal, path, frontier, closedNode, param_rtl = 0.5, param_lh = 2.28):
    
    ######### Initialization ############### 
    km = 0  
    data = {k: [float("inf"), float("inf")] for k in [(i, j) for i in range(grid.ix) for j in range(grid.iy)]}  
    overallPath = []
    Pnode = 0
    Tnode = 0                                                                      
    
    data[goal][1] = 0
    frontier[goal] = heuristic_distance(startP, goal)
    lastNode = startP
    robot.pos = startP
    robot.knownWorld = grid.walls[:]
    #robot.knownWorld = []
    ############## Main ####################
    Pnode = compute_shortest_path(robot, goal, data, frontier, km, closedNode, Pnode)
    print "Estimated LH: {0}".format(Pnode / heuristic_distance(startP, goal))
    modify_path(path, reconstruct_path_gradient(robot, data, goal))
    while robot.pos != goal:
        robot.pos = get_lowest_cost_node(robot, robot.pos, data)[0]
        Tnode += 1
        overallPath.append(robot.pos)
        changedNode = robot.detect_changes(grid)
        if changedNode[0] or changedNode[1]:
            print "Pnode: {0}, Tnode: {1}, Path: {2}, Ratio: {3}, Lheuristic: {4}".format(Pnode, Tnode, len(path), float(Tnode) / len(path), param_lh * heuristic_distance(robot.pos, goal))
            del closedNode[:]
            if float(Tnode) / len(path) > param_rtl: # -------------------- Ratio of traversed length
            #if  len(path) - Tnode <= param_lh * heuristic_distance(robot.pos, goal): # -------------- Linear Heuristic criteria
                km = 0
                frontier.clear()
                Pnode = 0
                Tnode = 0
                data = {k: [float("inf"), float("inf")] for k in [(i, j) for i in range(grid.ix) for j in range(grid.iy)]}  
                data[goal][1] = 0
                frontier[goal] = heuristic_distance(robot.pos, goal)
                lastNode = robot.pos
                for node in changedNode[0] + changedNode[1]: 
                    robot.update_cell(node) 
                Pnode = compute_shortest_path(robot, goal, data, frontier, km, closedNode, Pnode)
                print "Pnode: {0}, true".format(Pnode)
                modify_path(path, reconstruct_path_gradient(robot, data, goal))
            else:
                km += heuristic_distance(lastNode, robot.pos)
                lastNode = robot.pos
                for node in changedNode[0] + changedNode[1]: 
                    robot.update_cell(node)                # node -> v, next -> u          
                    for next in robot.detect_neighbor(node):
                        update_vertex(next, robot, goal, frontier, data, km)
                    update_vertex(node, robot, goal, frontier, data, km)             
                temp = compute_shortest_path(robot, goal, data, frontier, km, closedNode, 0)
                print temp, "False"   
                modify_path(path, reconstruct_path_gradient(robot, data, goal))        
        sleep(0.3)
    
###########################################################
#                  Utility                                #
###########################################################

def modify_path(path, newPath): # update global variable "path" to new path
    del path[:]
    path += newPath

def heuristic_distance(startP, goal): # calculate square root heuristic distance on grid
    return sqrt((startP[0] - goal[0])**2 + (startP[1] - goal[1])**2)

def heuristic_manhattan(startP, goal): # calculate manhattan heuristic distance on grid
    return abs(startP[0] - goal[0]) + abs(startP[1] - goal[1])

def reconstruct_path_gradient(robot, data, goal): # output the path from cost matrix
    current = robot.pos
    path = [current]
    if data[current][1] == float("inf"):
        return None
    while current != goal:
        current = get_lowest_cost_node(robot, current, data)[0]
        path.append(current)
    return path

def reconstruct_path(came_from, startP, goal): # output the path from search tree
    if came_from != None:
        path = []
        current = goal
        path.append(current)
        while current != startP:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
    else:
        return None