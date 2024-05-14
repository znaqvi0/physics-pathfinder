import random

import field
from vectors import Vec, norm, mag


def ccw(A, B, C):  # works for Vec because Vec has x and y attributes
    # https://stackoverflow.com/questions/3838329/how-can-i-check-if-two-segments-intersect
    return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x)


def intersect(A, B, C, D):
    """Return true if line segments AB and CD intersect"""
    return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)


def intersect_map(A, B, map: field.ObstacleMap):
    for poly in map.polygons:
        for i in range(len(poly) - 1):
            C, D = poly[i], poly[i + 1]
            if intersect(A, B, C, D):
                return True
    return False


def next_vector(point, obstacles, vec_generator, recur):
    if recur <= 0:
        return point
    vector = vec_generator()
    p1, p2 = point, vector
    in_field = lambda: field.LEFT_WALL < vector.x < field.RIGHT_WALL and field.BOTTOM_WALL < vector.y < field.TOP_WALL
    if not in_field() or intersect_map(p1, p2, obstacles):
        return next_vector(point, obstacles, vec_generator, recur-1)
    return vector


def vec_gauss_2d(vec, sigma):
    return Vec(random.gauss(vec.x, sigma), random.gauss(vec.y, sigma), vec.z)


def gauss_point_between(p1, p2, sigma):
    displacement = p2 - p1
    return vec_gauss_2d(p1 + norm(displacement) * random.uniform(0, mag(displacement)), sigma)


def probability(x):
    return random.uniform(0, 1) < x
