from Grid import SquareGrid, PriorityDict, map1, maze1
from Algorithm import astar_search, reconstruct_path, dlite_search
from threading import Thread
from robot import Robot
import pickle

res = 20
#res = 10
path = []
frontier = PriorityDict()
data = (None, None)
startP = (2, 0)
goal = (18, 15)

#startP = (9, 19)
#goal = (39, 19)

#startP = (17, 0)
#goal = (15, 18)

def setup():
    size(380, 400, P3D)
    #size(500, 400, P3D)
    #size(380, 380, P3D)
    global grid
    grid = maze1(res)
    #grid = map1(res)
    #grid = SquareGrid(width, height, res)
    global bot
    bot = Robot(2, 1, grid, startP)

def draw():
    grid.draw_grid(startP, goal, bot, None, None, frontier, path)
    

def mousePressed():
    curr = (mouseX // res, mouseY // res)
    if curr in grid.walls:
        grid.walls.remove(curr)
    else:
        grid.walls.append(curr)

def keyPressed():
    if key == 's':
        f = open("maze2.p", 'wb')
        pickle.dump(grid.walls, f)
        f.close()
    elif key == 'd':
        thread = Thread(target=dlite_path_analyst_thread)
        thread.start()
    elif key == 'p':
        thread = Thread(target=astar_path_analyst_thread)
        thread.start()
    elif key == 'r':
        thread = Thread(target=robot_move)
        thread.start()
    
    elif key == 'l':
        f = open('maze2.p', 'rb')
        grid.walls = pickle.load(f)
        f.close()

def astar_path_analyst_thread():
    global path
    global data
    data = astar_search(grid, startP, goal)
    path = reconstruct_path(data[0], startP, goal)
    

def robot_move():
    bot.isStop = False
    bot.pos = startP
    bot.path = path
    while not bot.isStop:
        print bot.get_data(grid)
        bot.traverse()

def dlite_path_analyst_thread():
    global path
    global frontier 
    dlite_search(grid, bot, startP, goal, path, frontier)

    