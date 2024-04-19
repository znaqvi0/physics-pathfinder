import field
from vectors import Vec


def ccw(A, B, C):  # works for Vec because Vec has x and y attributes
    # https://stackoverflow.com/questions/3838329/how-can-i-check-if-two-segments-intersect
    return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x)


def intersect(A, B, C, D):
    """Return true if line segments AB and CD intersect"""
    return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)


def intersect_map(A, B, map: field.ObstacleMap):
    for poly in map.polygons:
        for i in range(len(poly) - 1):
            C, D = poly[i], poly[i+1]
            if intersect(A, B, C, D):
                return True
    return False


def next_vector(point, obstacles, vec_generator):
    vector = vec_generator()
    p1, p2 = point, vector
    in_field = lambda: field.LEFT_WALL < vector.x < field.RIGHT_WALL and field.BOTTOM_WALL < vector.y < field.TOP_WALL
    if intersect_map(p1, p2, obstacles) or not in_field():
        return next_vector(point, obstacles, vec_generator)
    return vector


def segment_rect_intersect(A, B, rect_top_left: Vec, rect_width, rect_height):  # A, B: line segment points
    tl = rect_top_left
    tr = rect_top_left + Vec(rect_width, 0)
    bl = tl + Vec(0, -rect_height)
    br = tr + Vec(0, -rect_height)
    return intersect(A, B, tl, tr) or intersect(A, B, tr, bl) or intersect(A, B, bl, br) or intersect(A, B, br, tl)
