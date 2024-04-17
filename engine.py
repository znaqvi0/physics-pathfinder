import random

import field
from vectors import *

dt = 0.01  # 0.001


class Path:
    def __init__(self, start, target, n_waypoints, obstacles, color):
        self.start_point = start
        self.target_point = target
        self.num_waypoints = n_waypoints
        self.points = [start]
        self.obstacles = obstacles
        self.color = color
        self.fitness = -999999
        self.done = False
        self.populate()

    def populate(self):
        waypoints = []
        for i in range(self.num_waypoints):
            vec = lambda: Vec(random.uniform(field.LEFT_WALL, field.RIGHT_WALL),
                              random.uniform(field.BOTTOM_WALL, field.TOP_WALL))
            vector = vec()
            if type(self.obstacles) == field.ObstacleGrid:
                while self.obstacles.get_obstacle(vector.x, vector.y) == 1:
                    vector = vec()
            waypoints.append(vector)
        waypoints = sorted(waypoints, key=lambda x: mag(x - field.TARGET_POS), reverse=True)
        self.points.extend(waypoints)
        self.points.append(self.target_point)

    def find_intersections(self):
        if type(self.obstacles) == field.ObstacleGrid:
            for i in range(1, len(self.points)):
                p1, p2 = self.points[i-1], self.points[i]
                for row in range(self.obstacles.num_squares_x):
                    for col in range(self.obstacles.num_squares_y):
                        if self.obstacles.squares[row][col] == 1:
                            rect_top_left = self.obstacles.get_obstacle_pos(row, col)
                            if segment_rect_intersect(p1, p2, rect_top_left, self.obstacles.square_dimension, self.obstacles.square_dimension):
                                return rect_top_left  # approximates intersection

        elif type(self.obstacles) == field.ObstacleMap:
            for i in range(1, len(self.points)):
                p1, p2 = self.points[i-1], self.points[i]
                for poly in self.obstacles.polygons:
                    for i in range(1, len(poly)):
                        p3, p4 = poly[i - 1], poly[i]
                        if intersect(p1, p2, p3, p4):
                            return (p1+p2+p3+p4)/4  # intersection is somewhat close to this
        return self.points[-1]

    def calculate_fitness(self):
        intersects = self.find_intersections()
        intersection_score = 2 if mag(intersects - field.TARGET_POS) != 0 else 0
        length_score = sum([mag(self.points[i] - self.points[i - 1]) for i in range(len(self.points))])
        return -length_score - 100*intersection_score

    def update(self):
        self.fitness = self.calculate_fitness()
        self.done = True

    def varied_copy(self, sigma):
        # TODO chance to add/remove a point (change n_waypoints)
        path = Path(self.start_point, self.target_point, self.num_waypoints, self.obstacles, self.color)
        path.points = [self.start_point]
        for point in [pt for pt in self.points if pt not in [self.start_point, self.target_point]]:  # excluding start & end:  # excluding start & end
            vec = Vec(random.gauss(point.x, sigma), random.gauss(point.y, sigma))
            while True:
                within_bounds = field.LEFT_WALL < vec.x < field.RIGHT_WALL and field.BOTTOM_WALL < vec.y < field.TOP_WALL
                if type(self.obstacles) == field.ObstacleGrid:
                    not_hitting_obstacles = self.obstacles.get_obstacle(vec.x, vec.y) == 0
                else:
                    not_hitting_obstacles = True
                if within_bounds and not_hitting_obstacles:
                    path.points.append(vec)
                    break
                vec = Vec(random.gauss(point.x, sigma), random.gauss(point.y, sigma))
        path.points.append(self.target_point)
        return path


# https://stackoverflow.com/questions/3838329/how-can-i-check-if-two-segments-intersect
def ccw(A, B, C):  # works for Vec because Vec has x and y attributes
    return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x)


# Return true if line segments AB and CD intersect
def intersect(A, B, C, D):
    return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)


def segment_rect_intersect(A, B, rect_top_left: Vec, rect_width, rect_height):  # A, B: line segment points
    tl = rect_top_left
    tr = rect_top_left + Vec(rect_width, 0)
    bl = tl + Vec(0, -rect_height)
    br = tr + Vec(0, -rect_height)
    return intersect(A, B, tl, tr) or intersect(A, B, tr, bl) or intersect(A, B, bl, br) or intersect(A, B, br, tl)


class Ball:
    def __init__(self, position, velocity, radius, mass, color=(0, 0, 0)):
        self.v = velocity
        self.v0 = self.v.copy()
        self.pos = position
        self.pos0 = position
        self.a = Vec()
        self.r = radius
        self.m = mass
        self.color = color
        self.done = False
        self.t = 0
        self.max_height = 0
        self.range = 0

        self.fitness = 0

    def __repr__(self):
        return f"v0={self.v0}"

    def varied_copy_gaussian(self, sigma):
        ball = Ball(self.pos0,
                    Vec(random.gauss(self.v0.x, sigma), random.gauss(self.v0.y, sigma)),
                    self.r, self.m, self.color)
        # ball.v = Vec(random.gauss(self.v0.x, sigma), random.gauss(self.v0.y, sigma))
        ball.v0 = ball.v.copy()
        return ball

    def move(self):
        self.pos += self.v * dt
        self.a = self.force() / self.m
        self.v += self.a * dt

    def distance_from_target(self, target_pos):
        return mag(self.pos - target_pos)

    def calculate_fitness(self):
        score = -self.t - self.distance_from_target(field.TARGET_POS)
        return score

    def friction(self):
        return -0.015 * norm(self.v)

    def force(self):
        return self.friction()

    def collide_with_wall(self, wall_norm, sigma):
        randomized_wall_norm = wall_norm.rotate(random.gauss(0, sigma), degrees=True)
        self.pos -= self.v * dt
        self.v -= (2 * self.v.dot(wall_norm) * randomized_wall_norm)
        self.v *= 1 - 0.2 * abs(norm(self.v).dot(randomized_wall_norm))

    def check_walls(self):
        if self.pos.x < field.LEFT_WALL:
            self.collide_with_wall(field.LEFT_WALL_NORM, field.WALL_RANDOMNESS())
        elif self.pos.x > field.RIGHT_WALL:
            self.collide_with_wall(field.RIGHT_WALL_NORM, field.WALL_RANDOMNESS())
        if self.pos.y < field.BOTTOM_WALL:
            self.collide_with_wall(field.BOTTOM_WALL_NORM, field.WALL_RANDOMNESS())
        elif self.pos.y > field.TOP_WALL:
            self.collide_with_wall(field.TOP_WALL_NORM, field.WALL_RANDOMNESS())

    def update(self):
        if mag(self.v) > 0.005 and not self.in_hole():  # TODO and not self.close_enough() (to target)
            self.check_walls()
            self.move()
            self.t += dt
        else:
            if not self.done:
                self.fitness = self.calculate_fitness()
                string = ""
                string += str(self.fitness)
                # print(string)
            self.done = True
