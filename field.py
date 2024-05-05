import random

import numpy as np

from vectors import *

WIDTH = 16.4846
HEIGHT = 8.1026
START_POS = Vec(WIDTH * 0.08, HEIGHT * 0.675)
TARGET_POS = Vec(WIDTH * 0.95, HEIGHT * 0.1)

LEFT_WALL = 0
BOTTOM_WALL = 0
RIGHT_WALL = WIDTH
TOP_WALL = HEIGHT

LEFT_WALL_NORM = Vec(1, 0)
RIGHT_WALL_NORM = Vec(-1, 0)
TOP_WALL_NORM = Vec(0, -1)
BOTTOM_WALL_NORM = Vec(0, 1)

WALL_RANDOMNESS = lambda: random.gauss(0, 0.1)

GRID_DIMENSION = 1  # HEIGHT // 8


class ObstacleGrid:
    def __init__(self, square_dimension):
        self.square_dimension = square_dimension
        self.num_squares_x = int(WIDTH // square_dimension) + 1
        self.num_squares_y = int(HEIGHT // square_dimension) + 1
        self.squares = np.zeros((self.num_squares_x, self.num_squares_y))

    def add_obstacle(self, x, y):
        if LEFT_WALL < x < RIGHT_WALL and BOTTOM_WALL < y < TOP_WALL:
            self.squares[int(x * self.num_squares_x // WIDTH)][int(y * self.num_squares_y // HEIGHT)] = 1

    def remove_obstacle(self, x, y):
        if LEFT_WALL < x < RIGHT_WALL and BOTTOM_WALL < y < TOP_WALL:
            self.squares[int(x * self.num_squares_x // WIDTH)][int(y * self.num_squares_y // HEIGHT)] = 0

    def get_obstacle(self, x, y):
        if LEFT_WALL < x < RIGHT_WALL and BOTTOM_WALL < y < TOP_WALL:
            return self.squares[int(x * self.num_squares_x // WIDTH)][int(y * self.num_squares_y // HEIGHT)]
        return 0

    def get_obstacle_pos(self, i_x, i_y):
        if i_x < self.num_squares_x and i_y + 1 < self.num_squares_y:
            return Vec(i_x * self.square_dimension, (i_y + 1) * self.square_dimension)
        return Vec(self.square_dimension, self.square_dimension)


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


if __name__ == "__main__":
    grid = ObstacleGrid(GRID_DIMENSION)
    grid.add_obstacle(7.5, 5)
    print(grid.squares.T)


class Wall:
    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2
        self.x = self.p1.x
        self.y = self.p1.y

