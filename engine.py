import random

import field
from util import intersect_map, next_vector, vec_gaussian_2d, probability
from vectors import *

dt = 0.01  # 0.001


class Path:
    def __init__(self, start, target, obstacles, color):
        self.start_point = start
        self.target_point = target
        self.points = []
        self.obstacles = obstacles
        self.color = color
        self.fitness = -999999
        self.done = False
        self.populate()

    def populate(self):
        waypoints = [self.start_point]
        self.points = []
        i = 0
        vec_generator = lambda: Vec(random.uniform(field.LEFT_WALL, field.RIGHT_WALL), random.uniform(field.BOTTOM_WALL, field.TOP_WALL))
        while intersect_map(waypoints[i], self.target_point, self.obstacles):  # and i < self.num_waypoints
            waypoints.append(next_vector(waypoints[i], self.obstacles, vec_generator, 900))
            i += 1
        self.points.extend(waypoints)
        self.points.append(self.target_point)

    def intersects_map(self):
        for i in range(len(self.points) - 1):
            p1, p2 = self.points[i], self.points[i+1]
            if intersect_map(p1, p2, self.obstacles):
                return True
        return False

    def find_intersections(self):
        for i in range(1, len(self.points)):
            p1, p2 = self.points[i-1], self.points[i]
            if intersect_map(p1, p2, self.obstacles):
                return (p1+p2)/2  # intersection is somewhat close to this
        return self.points[-1]

    def calculate_fitness(self):
        length_score = sum([mag(self.points[i] - self.points[i - 1]) for i in range(1, len(self.points))])
        return -length_score

    def update(self):
        self.fitness = self.calculate_fitness()
        self.done = True

    def varied_copy(self, sigma, dropout=True):
        path = Path(self.start_point, self.target_point, self.obstacles, self.color)
        path.points = [self.start_point]
        vec = lambda: vec_gaussian_2d(point, sigma)

        keep_chance = 0.8 if dropout and sigma > 0.0005 else 1
        add_chance = 1 - keep_chance

        pts = [pt for pt in self.points if pt not in [self.start_point, self.target_point]]  # excluding start & end
        for point in pts:
            if probability(keep_chance):
                path.points.append(next_vector(point, self.obstacles, vec, 5))
            if probability(add_chance):
                vector = next_vector(point, self.obstacles, vec, 5)
                if mag(point - vector) > 0.005:  # prevent placing points nearly on top of each other
                    path.points.append(vector)
        path.points.append(self.target_point)
        if path.intersects_map():
            path = self.varied_copy(sigma/10, dropout)  # try again on failure
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

    def varied_copy_gaussian(self, sigma):
        ball = Ball(self.pos0,
                    Vec(random.gauss(self.v0.x, sigma), random.gauss(self.v0.y, sigma)),
                    self.r, self.m, self.color)
        ball.v0 = ball.v.copy()
        return ball

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
