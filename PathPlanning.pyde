from Grid import SquareGrid
from Algorithm import astar_search, reconstruct_path

res = 10
path = None
startP = (9, 19)
goal = (39, 19)
def setup():
    size(500, 400, P3D)
    global grid
    grid = SquareGrid(width, height, res)

def draw():
    grid.draw_grid(startP, goal, None, None, path)
    

def mousePressed():
    curr = (mouseX // res, mouseY // res)
    if curr in grid.walls:
        grid.walls.remove(curr)
    else:
        grid.walls.append(curr)

def keyPressed():
    global path
    path = reconstruct_path(astar_search(grid, startP, goal), startP, goal)