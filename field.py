import random

from vectors import *

# TODO change
# TODO obstacle class (similar to moat)
WIDTH = 16.4846
HEIGHT = 8.1026
START_POS = Vec(WIDTH/10, HEIGHT/2)
TARGET_POS = Vec(WIDTH * 9/10, HEIGHT * 3/4)

LEFT_WALL = 0
BOTTOM_WALL = 0
RIGHT_WALL = WIDTH
TOP_WALL = HEIGHT

LEFT_WALL_NORM = Vec(1, 0)
RIGHT_WALL_NORM = Vec(-1, 0)
TOP_WALL_NORM = Vec(0, -1)
BOTTOM_WALL_NORM = Vec(0, 1)

WALL_RANDOMNESS = lambda: random.gauss(0, 0.1)


class Wall:
    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2
        self.x = self.p1.x
        self.y = self.p1.y

