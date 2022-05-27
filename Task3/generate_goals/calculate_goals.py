#!/usr/bin/python3

from defer import return_value
import numpy
import re
from matplotlib import pyplot
import cv2
import numpy as np


grid = np.zeros((1, 1))
grid_goals = []
image = np.zeros((2,2))
image_rgb = np.zeros((2,2))


def read_pgm(filename, byteorder='>'):
    """Return image data from a raw PGM file as numpy array.
    Format specification: http://netpbm.sourceforge.net/doc/pgm.html
    """
    with open(filename, 'rb') as f:
        buffer = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)
    return numpy.frombuffer(buffer,
                            dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                            count=int(width)*int(height),
                            offset=len(header)
                            ).reshape((int(height), int(width)))


def create_everything():
    global image
    global image_rgb
    image = read_pgm("src/exercise7/scripts/map_ok.pgm", byteorder='<')
    image_rgb = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    image_rgb = image_rgb[199:298, 205:327]
    image_rgb_height = len(image_rgb) # 99
    image_rgb_width= len(image_rgb[0]) # 122
    image = image[199:298, 205:327]
    
    global grid
    global grid_goals
    grid = create_grid(image_rgb_height, image_rgb_width)
    # print(grid)


def calculate_goals():
    global grid
    global grid_goals
    # začnemo na -1, 0 (spodaj skoraj čisto levo)
    start_index = (3, 0)
    find_path(start_index[0], start_index[1], len(grid), len(grid[0]))
    
    # print(grid)
    # print(grid_goals)
    image_coordinates = []
    for goal in grid_goals:
        y, x = grid_coordinates_to_image(goal[0], goal[1])
        image_coordinates.append((y, x))
        image_rgb[y, x, :] = 0
        image_rgb[y, x, 2] = 255
    cv2.imwrite('src/exercise7/scripts/kvadrati/goali.png', image_rgb)
    
    # pretvarjanje iz koordinat slike v koordinate za robota
    robot_coordinates = []
    for goal in image_coordinates:
        y_r, x_r = image_coordinates_to_robot(goal[0], goal[1])
        # pr goalih je glih obratno - najprej x, potem y
        robot_coordinates.append((x_r, y_r))
        # robot_coordinates.append((y_r, x_r))
        
    return robot_coordinates
    
    
def find_path(y, x, rows, cols):
    global grid
    # print(y, ",", x)
    
    # če pridemo ven iz grida
    if y < 0 or y >= rows or x < 0 or x >= cols:
        # print("     oob")
        return
    
    # če je trenutno mesto že bilo obiskano
    elif grid[y, x] == -1 or grid[y, x] == 1:
        # print("     visited")
        return

    # označimo, da smo obiskali trenutno mesto
    grid[y, x] = 1
    
    # če je soseščina prosta, dodamo te koordinate kot goal
    if check_vicinity(y, x):
        # print("je šla skoz")
        global grid_goals
        grid_goals.append((y, x))
        
    find_path(y+1, x, rows, cols)   # dol
    find_path(y, x+1, rows, cols)   # desno
    find_path(y, x-1, rows, cols)   # levo
    find_path(y-1, x, rows, cols)   # gor
    
    
def check_vicinity(y, x):
    global grid
    y_img, x_img = grid_coordinates_to_image(y, x)
    partial_image = image[y_img-5:y_img+5, x_img-5:x_img+5]
    # print(partial_image)
    for y in range(len(partial_image)):
        for x in range(len(partial_image[0])):
            if partial_image[y, x] < 254:
                return False
    return True
    

def grid_coordinates_to_image(y, x, scaling_factor=20):
    offset = int(scaling_factor/2)
    y_img = y*scaling_factor + offset
    x_img = x*scaling_factor + offset
    return y_img, x_img


# bolj gosti goali => manjši scaling factor
def create_grid(image_height, image_width, scaling_factor=20):
    rows = round(image_height/scaling_factor)
    cols = round(image_width/scaling_factor)
    grid = np.zeros((rows, cols))
    return grid


def image_coordinates_to_robot(y, x):
    step = 1.96 / 39    # prebrano iz rviza (39 col je 1.96 v rvizu)
    row_offset = 69    # zamik do koordinat 0,0
    col_offset = 39    # zamik do koordinat 0,0
    y_robot = (row_offset-y) * step
    x_robot = (x-col_offset) * step
    return round(y_robot, 2), round(x_robot, 2)


# funkcija za import v main (move) fajlu
def goals():
    create_everything()
    goals = calculate_goals()
    return goals


if __name__ == "__main__":
    create_everything()
    goals = calculate_goals()
    print(goals)
    