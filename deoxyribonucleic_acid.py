import sys  # most commonly used to turn the interpreter off (shut down your game)

import pygame as p

import field
from engine import *
from families import Family

p.init()

WIDTH = 800
HEIGHT = 800
font = p.font.SysFont('Monocraft', 20)

screen_color = (100, 100, 100)
ground_color = (150, 150, 150)  # (50, 200, 100)
screen = p.display.set_mode((WIDTH, HEIGHT))
# screen.fill((150, 210, 255))
screen.fill(screen_color)
p.display.set_caption('window')

scale = 40  # 200
origin = x0, y0 = WIDTH / 2 - field.WIDTH/2*scale, HEIGHT / 2 + field.HEIGHT/2*scale  # This is the new origin

field_img = p.image.load("frc2024.png").convert_alpha()  # https://www.chiefdelphi.com/t/2024-crescendo-top-down-field-renders/447764
field_img = p.transform.scale(field_img, (field.WIDTH * scale, field.HEIGHT * scale))

# obstacles = field.ObstacleGrid(0.75)  # this is passed in as a reference; changing this changes all paths that use it
obstacles = field.ObstacleMap()


def ball_xy(ball):
    return float(origin[0] + ball.pos.x * scale), float(origin[1] - ball.pos.y * scale)


def draw_ball(ball):
    p.draw.circle(screen, ball.color, ball_xy(ball), max(ball.r * scale, 1))


def make_display(text, top_left, text_color=(255, 255, 255), bg_color=None):
    display = font.render(text, True, text_color, bg_color)
    display_rect = display.get_rect()
    display_rect.topleft = top_left
    return display, display_rect


def draw_text(text, top_left, text_color=(255, 255, 255)):
    display, display_rect = make_display(text, (0, 0), text_color=text_color, bg_color=None)
    display_rect.topleft = top_left
    screen.blit(display, display_rect)


def draw_line(p1, p2, color=(0, 0, 0)):
    p.draw.line(screen, color, (x0 + p1.x * scale, y0 - p1.y * scale),
                (x0 + p2.x * scale, y0 - p2.y * scale))


def draw_rect(color, left_x, top_y, width, height):
    p.draw.rect(screen, color, (x0 + left_x * scale, y0 - top_y * scale, width * scale, height * scale))


def draw_translucent(color, alpha, pos, width, height):
    s = p.Surface((width * scale, height * scale))
    s.set_alpha(alpha)
    s.fill(color)
    screen.blit(s, (x0 + pos.x * scale, y0 - pos.y * scale))


def draw_obstacles(_obstacles: field.ObstacleMap):
    for poly in _obstacles.polygons:
        for i in range(1, len(poly)):
            draw_line(poly[i-1], poly[i], (255, 0, 0))


def draw_course():
    screen.fill(screen_color)
    draw_rect((0, 200, 50), field.LEFT_WALL, field.TOP_WALL, field.WIDTH, field.HEIGHT)
    screen.blit(field_img, (x0 + field.LEFT_WALL*scale, y0 - field.TOP_WALL*scale))
    draw_obstacles(obstacles)
    draw_ball(Ball(field.START_POS, Vec(), 0.2, 1, (255, 255, 255)))
    draw_ball(Ball(field.TARGET_POS, Vec(), 0.2, 1, (255, 255, 255)))


def draw_path(path):
    for i in range(1, len(path.points)):
        draw_line(path.points[i-1], path.points[i], path.color)


def random_path():
    return Path(field.START_POS, field.TARGET_POS, 10, obstacles,
                color=(random.uniform(0, 255), random.uniform(0, 255), random.uniform(0, 255)))


def get_mouse_xy_meters():
    x, y = p.mouse.get_pos()
    newx, newy = x - x0, y0 - y
    newx *= 1/scale  # scale converts meters to pixels
    newy *= 1/scale  # 1/scale converts pixels to meters
    return newx, newy


# constants (change to robot dimensions/mass)
pos0 = field.START_POS
r = 0.021335
m = 0.045

initial_population = 1000
population = 20
num_families = 3

sigma = 2
sigma_rate = 0.98

generation = 1

families = []


def all_families_done(families):
    for family in families:
        if not family.all_done():
            return False
    return True


draw_course()
running = False
t = 0
while True:
    for event in p.event.get():
        if event.type == p.QUIT:  # this refers to clicking on the "x"-close
            p.quit()
            sys.exit()
        elif event.type == p.KEYDOWN:
            if event.key == p.K_SPACE:
                running = not running
                families = []
                for i in range(num_families):  # populate once all obstacles are drawn
                    families.append(Family(population // num_families, sigma, sigma_rate).populate(seed=random_path()))

    mouse_pressed = p.mouse.get_pressed(num_buttons=3)
    if mouse_pressed[0]:
        obst_x, obst_y = get_mouse_xy_meters()
        obstacles.add_point(obst_x, obst_y)
        draw_course()
    # elif mouse_pressed[2]:  # TODO remove closest point to cursor
    #     obst_x, obst_y = get_mouse_xy_meters()
    #     obstacles.remove_obstacle(obst_x, obst_y)
    #     draw_course()
    elif mouse_pressed[2]:
        if len(obstacles.polygons[-1]) > 0:
            obstacles.new_poly()

    if running:
        for i in range(1):  # steps multiple times every frame
            for family in families:
                family.update()

        families = sorted(families, key=lambda fam: fam.family_score, reverse=True)

        if len(families) > 1 and families[0].sigma < 0.005:
            if generation % 1 == 0:  # kill off a family every _ generations
                families.remove(families[-1])

                for family in families:
                    family.population = population // len(families)

        # best_path = sorted(families, key=lambda fam: fam.best_path.fitness, reverse=True)[0].best_path
        screen.fill(screen_color)
        draw_course()
        for family in families:
            family.paths = family.next_gen()

            for path in family.paths:
                draw_path(path)

        draw_text("sigma: %.10f" % families[0].sigma, (20, 20))
        draw_text("best family score: %.5f" % families[0].family_score, (20, 40))
        draw_text("number of families: %.0i" % len(families), (20, 60))
        draw_text("generation: %.0i" % families[0].generations_passed, (20, 80))

    p.display.flip()
    p.time.Clock().tick()  # caps frame rate at 100
