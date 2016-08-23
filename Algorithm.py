from Queue import PriorityQueue

def heuristic_distance(startP, goal):
    return sqrt((startP[0] - goal[0])**2 + (startP[1] - goal[1])**2)

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
        
    return came_from if current == goal else None

def dstar_search(grid, startP, goal):
    

def reconstruct_path(came_from, startP, goal):
    if came_from != None:
        path = []
        current = goal
        path.append(current)
        while current != startP:
            current = came_from[current]
            path.append(current)
        return path
    else:
        return None