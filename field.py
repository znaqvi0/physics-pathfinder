import random

import numpy as np

from vectors import *

WIDTH = 25  # 16.4846
HEIGHT = 14  # 8.1026
START_POS = Vec(WIDTH * 0.08, HEIGHT * 0.675)
TARGET_POS = Vec(WIDTH * 0.95, HEIGHT * 0.1)

LEFT_WALL = 0
BOTTOM_WALL = 0
RIGHT_WALL = WIDTH
TOP_WALL = HEIGHT


class ObstacleMap:
    def __init__(self):
        self.polygons = []
        self.new_poly()

    def add_point(self, x, y):
        if len(self.polygons[-1]) > 0:
            if mag(self.polygons[-1][-1] - Vec(x, y)) > 0.5:
                self.polygons[-1].append(Vec(x, y))
        else:
            self.polygons[-1].append(Vec(x, y))

    def new_poly(self):
        self.polygons.append([])

    def undo(self):
        if len(self.polygons[-1]) > 0:
            self.polygons[-1].remove(self.polygons[-1][-1])

    def reset(self):
        self.__init__()
