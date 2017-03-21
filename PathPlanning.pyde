#######################################################################################################
# Author: An T. Le, DOB: 11/11/1997
# Organization: Vietnamese-German University
# Date Updated: March 21th, 2017
# Date Created: July 2nd, 2016
# Application: Occupancy Grid Library and Search-based Path Planning Library 
#######################################################################################################


from Grid import SquareGrid, PriorityDict, map1, maze1
from Algorithm import astar_search, reconstruct_path, dlite_search
from threading import Thread
from robot import Robot
from MazeGenerator import MazeGenerator


################################ User Configuration ###################################################

res = 10 # set map resolution (10 means 10 bit per square grid)

startP = (1, 1) # set starting and goal position of the robot (origin of coordinate is at top left of the window)
#goal = (30, 10)

sizeX = 510 # set the size of the environment
sizeY = 510

############################### Main Program ###########################################################
#global variables
path = []
frontier = PriorityDict()
data = (None, None)
closedNode = []

#Main
def setup():
    size(sizeX, sizeY, P3D) # set the size of the environnment
    
    global goal
    goal = (width / res - 2, height / res - 2) # always set goal at bottom right of the map
    
    global maze
    maze = MazeGenerator(width, height, res, (1, 1)) # initialize map 
    maze.generate() # generate maze
    
    global bot
    bot = Robot(2, 1, maze.grid, startP) # initialize robot perception
                
def draw():
    maze.grid.draw_grid(startP, goal, bot, None, None, None, path, closedNode) # continuously draw map and robot conditions in real-time  

def mousePressed():
    curr = (mouseX // res, mouseY // res) # identify a grid position when clicked on the windows
    if curr in maze.grid.walls: # on-off rountine, if it is an obstacle, then remove it and vice versa.
        maze.grid.walls.remove(curr) 
    else:
        maze.grid.walls.append(curr)

def keyPressed():
    if key == 'd': # trigger d*lite algorithm with LHD criteria
        thread = Thread(target=dlite_path_analyst_thread)
        thread.start()
    if key == 'r': # trigger d*lite algorithm with Ratio criteria
        thread = Thread(target=dlr_path_analyst_thread)
        thread.start()
    elif key == 'a': # trigger a* algorithm with LHD criteria
        thread = Thread(target=astar_path_analyst_thread)
        thread.start()
    elif key == 'b': # move the robot along the calculated path
        thread = Thread(target=robot_move)
        thread.start()


def astar_path_analyst_thread(): # define thread for a* algorithm
    global path
    global data
    data = astar_search(maze.grid, startP, goal)
    path = reconstruct_path(data[0], startP, goal)
    

def robot_move(): # define thread for robot move along calculated path
    bot.isStop = False
    bot.pos = startP
    bot.path = path
    while not bot.isStop:
        print bot.get_data(grid)
        bot.traverse()

def dlite_path_analyst_thread(): # define thread for d*lite algorithm with LHD criteria
    global path
    global frontier 
    dlite_search(maze.grid, bot, startP, goal, path, frontier, closedNode, 1)
    
def dlr_path_analyst_thread(): # define thread for d*lite algorithm with Ratio criteria
    global path
    global frontier 
    dlite_search(maze.grid, bot, startP, goal, path, frontier, closedNode, 0.5)


    