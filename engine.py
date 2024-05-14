import random

import field
from util import intersect_map, next_vector, vec_gauss_2d, probability, gauss_point_between
from vectors import *

dt = 0.01  # 0.001


class Path:
    def __init__(self, start, target, obstacles, color, populate=False):
        self.start_point = start
        self.target_point = target
        self.points = []
        self.obstacles = obstacles
        self.color = color
        self.fitness = -999999
        self.done = False
        if populate:
            self.populate()

    def populate(self):
        waypoints = [self.start_point]
        self.points = []
        i = 0
        vec_generator = lambda: vec_gauss_2d(waypoints[i], sigma=10)
        while intersect_map(waypoints[i], self.target_point, self.obstacles):
            vector = next_vector(waypoints[i], self.obstacles, vec_generator, 900)
            if mag(waypoints[i] - vector) > 0.1:
                waypoints.append(vector)
                i += 1

        j = 0
        new_waypoints = [self.start_point]
        sort_waypoints = sorted(waypoints, key=lambda pt: mag(pt - waypoints[j]), reverse=True)
        while intersect_map(new_waypoints[j], self.target_point, self.obstacles):
            for k in reversed(range(len(sort_waypoints))):
                if not intersect_map(new_waypoints[j], waypoints[k], self.obstacles):
                    new_waypoints.append(waypoints[k])
                    j += 1
                    break

        self.points.extend(new_waypoints)
        self.points.append(self.target_point)

    def intersects_map(self):
        for i in range(len(self.points) - 1):
            p1, p2 = self.points[i], self.points[i + 1]
            if intersect_map(p1, p2, self.obstacles):
                return True
        return False

    def calculate_fitness(self):
        length_score = sum([mag(self.points[i] - self.points[i - 1]) for i in range(1, len(self.points))])
        num_points_score = len(self.points) * 0.02
        return -length_score - num_points_score

    def update(self):
        self.fitness = self.calculate_fitness()
        self.done = True

    def varied_copy(self, sigma, dropout=True, dropout_val=0.5):
        path = Path(self.start_point, self.target_point, self.obstacles, self.color)
        path.points = [self.start_point]

        keep_chance = dropout_val if dropout else 1
        add_chance = 1 - keep_chance

        pts = self.points[:]
        for i in range(len(pts)-1):  # exclude target point
            point = pts[i]

            if probability(keep_chance):
                if point != self.start_point:  # exclude start point
                    path.points.append(next_vector(point, self.obstacles, lambda: vec_gauss_2d(point, sigma), 50))

            if probability(add_chance):
                vector = next_vector(point, self.obstacles,
                                     lambda: gauss_point_between(point, pts[i + 1], sigma), 50)
                if mag(point - vector) > 0.01:
                    path.points.append(vector)

        path.points.append(self.target_point)
        if path.intersects_map():
            path = self.varied_copy(sigma / 2, dropout, 1 - (1 - dropout_val) * 0.5)  # try again on failure
        return path


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

    def __repr__(self):
        return f"v0={self.v0}"

    def move(self):
        self.pos += self.v * dt
        self.a = self.force() / self.m
        self.v += self.a * dt

    def distance_from_target(self, target_pos):
        return mag(self.pos - target_pos)

    def force(self):  # path following algorithm (force toward next point + wheel friction)
        return Vec()

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
        if mag(self.v) > 0.005:  # check if close enough to target
            self.check_walls()
            self.move()
            self.t += dt
            return
        elif not self.done:
            # code for when ball is done
            self.done = True
