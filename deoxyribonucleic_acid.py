import sys  # most commonly used to turn the interpreter off (shut down your game)

import pygame as p

import obstacle_presets
from engine import *
from families import Family
import matplotlib.pyplot as plt

p.init()

WIDTH = 1200
HEIGHT = 800
font = p.font.SysFont('Monocraft', 20)

screen_color = (100, 100, 100)
ground_color = (150, 150, 150)  # (50, 200, 100)
screen = p.display.set_mode((WIDTH, HEIGHT))
screen.fill(screen_color)
p.display.set_caption('window')

scale = 40  # 200
origin = x0, y0 = WIDTH / 2 - field.WIDTH/2*scale, HEIGHT / 2 + field.HEIGHT/2*scale  # This is the new origin

# frc field: https://www.chiefdelphi.com/t/2024-crescendo-top-down-field-renders/447764
field_img = p.image.load("maze.png").convert_alpha()
field_img = p.transform.scale(field_img, (field.WIDTH * scale, field.HEIGHT * scale))

obstacles = field.ObstacleMap()


def ball_xy(ball):
    """converts meter coordinates to pixel coordinates"""
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


def draw_line(p1, p2, color=(0, 0, 0), width=1):
    p.draw.line(screen, color, (x0 + p1.x * scale, y0 - p1.y * scale),
                (x0 + p2.x * scale, y0 - p2.y * scale), width=width)


def draw_rect(color, left_x, top_y, width, height):
    p.draw.rect(screen, color, (x0 + left_x * scale, y0 - top_y * scale, width * scale, height * scale))


def draw_obstacles(_obstacles: field.ObstacleMap):
    for poly in _obstacles.polygons:
        for i in range(1, len(poly)):
            draw_line(poly[i-1], poly[i], (255, 0, 0))


def draw_course(start, target):
    screen.fill(screen_color)
    draw_rect((50, 50, 50), field.LEFT_WALL, field.TOP_WALL, field.WIDTH, field.HEIGHT)
    # screen.blit(field_img, (x0 + field.LEFT_WALL*scale, y0 - field.TOP_WALL*scale))
    draw_obstacles(obstacles)
    draw_ball(Ball(start, Vec(), 0.2, 1, (255, 255, 255)))
    draw_ball(Ball(target, Vec(), 0.2, 1, (255, 255, 255)))


def draw_path(path):
    for i in range(1, len(path.points)):
        draw_line(path.points[i-1], path.points[i], path.color, width=3)


def random_path(start, target):
    return Path(start, target, obstacles,
                color=(random.uniform(100, 255), random.uniform(100, 255), random.uniform(100, 255)), populate=True)


def get_mouse_xy_meters():
    x, y = p.mouse.get_pos()
    newx, newy = x - x0, y0 - y
    newx *= 1/scale  # scale converts meters to pixels
    newy *= 1/scale  # 1/scale converts pixels to meters
    return newx, newy


def graph_scores(data):
    """plots average path length as a function of the generation"""
    labels = []
    scores = []
    for gen, score in data:
        labels.append(gen)
        scores.append(score)

    fig1, ax1 = plt.subplots()
    ax1.plot(labels, scores)

    plt.xlabel("generation")
    plt.ylabel("average length")

    plt.show()


# constants
preset = obstacle_presets.none

pos0 = preset.start
target_pos = preset.target

obstacles.polygons = preset.polygons

population = 50
num_families = population//10

sigma = 5
sigma_rate = 0.8
generation = 1

families = []

score_data = []
plot_scores = True


draw_course(pos0, target_pos)
running = False
populate = True
while True:
    for event in p.event.get():
        if event.type == p.QUIT:  # this refers to clicking on the "x"-close
            p.quit()
            sys.exit()
        elif event.type == p.KEYDOWN:
            # --------------------preset choosing--------------------
            if event.key == p.K_0:
                preset = obstacle_presets.none
            if event.key == p.K_1:
                preset = obstacle_presets.maze
            if event.key == p.K_2:
                preset = obstacle_presets.circles2
            if event.key == p.K_3:
                preset = obstacle_presets.spiral2

            pos0 = preset.start
            target_pos = preset.target

            obstacles.polygons = preset.polygons
            # -------------------------------------------------------
            if event.key == p.K_SPACE:  # start the program
                running = not running
                if populate:
                    families = []
                    for i in range(num_families):  # populate once all obstacles are drawn
                        families.append(Family(population // num_families, sigma, sigma_rate).populate(seed=random_path(pos0, target_pos)))
                    populate = False

            if event.key == p.K_z:
                obstacles.undo()
            if event.key == p.K_r:
                obstacles.reset()
            if event.key == p.K_n:
                obstacles.new_poly()

            draw_course(pos0, target_pos)

    # edit obstacles based on mouse input
    mouse_pressed = p.mouse.get_pressed(num_buttons=3)
    if mouse_pressed[0]:
        obst_x, obst_y = get_mouse_xy_meters()
        obstacles.add_point(obst_x, obst_y)
        draw_course(pos0, target_pos)
    elif mouse_pressed[2]:
        if len(obstacles.polygons[-1]) > 0:
            obstacles.new_poly()

    if running:
        families = sorted(families, key=lambda fam: fam.family_score, reverse=True)

        if len(families) > 1 and families[0].sigma < 0.01:
            if generation % 1 == 0:  # kill off a family every _ generations
                families.remove(families[-1])

                for family in families:
                    family.population = min(30, population // len(families))

        screen.fill(screen_color)
        draw_course(pos0, target_pos)
        # next generation
        for family in families:
            family.paths = family.next_gen()
            family.update()

            for path in family.paths:
                draw_path(path)

        if plot_scores:  # record score data
            if families[0].generation % 1 == 0:
                score_data.append([families[0].generation, abs(families[0].family_score)])  # abs(score) = path length
            if families[0].generation == 100:
                break

        draw_text("generation: %.0i" % families[0].generation, (20, 20))
        draw_text("best length: %.3f meters" % families[0].best_path.length(), (20, 40))

    p.display.flip()
    p.time.Clock().tick()

print(obstacles.polygons)  # copy and paste the result of this into a new preset
if plot_scores:
    graph_scores(score_data[1:])
