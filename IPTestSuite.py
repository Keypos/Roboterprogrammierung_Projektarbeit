# coding: utf-8

"""
This code is part of the course "Introduction to robot path planning" (Author: Bjoern Hein).
It gathers all visualizations of the investigated and explained planning algorithms.
License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

import random
from IPBenchmark import Benchmark
from IPEnvironment import CollisionChecker
from shapely.geometry import Point, Polygon, LineString
import shapely.affinity
import math
import numpy as np


benchList = list()


# -----------------------------------------
# Ab hier Benchmarks aus vorherigen Vorlesungen
trapField = dict()
trapField["obs1"] = LineString(
    [(6, 18), (6, 8), (16, 8), (16, 18)]).buffer(1.0)
description = "Following the direct connection from goal to start would lead the algorithm into a trap."
benchList.append(Benchmark("Trap", CollisionChecker(
    trapField), [[10, 15]], [[10, 1]], description, 2))

# -----------------------------------------
bottleNeckField = dict()
bottleNeckField["obs1"] = Polygon([(-5, -5),(11,-5), (11, 30),(-5,30)])
bottleNeckField["obs2"] = Polygon([(13, -5),(30,-5), (30, 30),(13,30)])
#bottleNeckField["obs2"] = LineString([(13, 13), (23,13)]).buffer(10)
description = "Planer has to find a narrow passage."
benchList.append(Benchmark("Bottleneck", CollisionChecker(bottleNeckField) , [[4,15]], [[18,1]], description, 2))

# -----------------------------------------
fatBottleNeckField = dict()
fatBottleNeckField["obs1"] = Polygon(
    [(0, 8), (11, 8), (11, 15), (0, 15)]).buffer(.5)
fatBottleNeckField["obs2"] = Polygon(
    [(13, 8), (24, 8), (24, 15), (13, 15)]).buffer(.5)
description = "Planer has to find a narrow passage with a significant extend."
benchList.append(Benchmark("Fat bottleneck", CollisionChecker(
    fatBottleNeckField), [[4, 21]], [[18, 1]], description, 2))

# -----------------------------------------
trapField = dict()
for i in range(10, 1300, 10):
    radius = 1.0 * (i / 500.0)
    width = 1.0 * (i / 5000.0)
    trapField["obsA"+str(i/10)] = Point([(10 - np.cos(np.deg2rad(i))
                                        * radius, 10 - np.sin(np.deg2rad(i))*radius)]).buffer(width)
    trapField["obsB"+str(i/10)] = Point([(15 + np.sin(np.deg2rad(i))
                                        * radius, 15 + np.cos(np.deg2rad(i))*radius)]).buffer(width)
trapField["obsC"] = LineString(
    [(5, 0.5), (5, 10), (15, 20), (20, 20)]).buffer(0.5)

start = [[10, 10]]
goal = [[15, 15]]

description = "Two spirals block the way from start to goal."
benchList.append(Benchmark("Spirals", CollisionChecker(
    trapField), start, goal, description, 4))


# -----------------------------------------
trapField = dict()
form1 = Point(11.5, 11.5).buffer(10.0)
form2 = LineString([(0, 11.5), (11.5, 11.5), (16, 5), (16, 18)]).buffer(1.0)

trapField["obs1"] = form1.difference(form2)

start = [[1, 5]]
goal = [[16, 18]]

description = "The robot needs to find the entrance to the circle."
benchList.append(Benchmark("Entrance", CollisionChecker(
    trapField), start, goal, description, 2))

# -----------------------------------------
trapField = dict()

for i in range(1, 9):
    circle = Point(11.5, 11.5).buffer(1.0 * i)
    innercircle = Point(11.5, 11.5).buffer((1.0 * i) - 0.5)
    entrance = LineString([(11.5, 11.5), (11.5 + np.sin(np.deg2rad(360.0 / i) * (-1)**i) * (
        1.0 * i), 11.5 + np.cos(np.deg2rad(360.0 / i) * (-1)**i) * (1.0 * i))]).buffer(0.2)
    ring = circle.difference(innercircle)
    trapField["obs"+str(i)] = ring.difference(entrance)

start = [[5, 5]]
goal = [[11.5, 11.5]]

description = "The robot needs to find all entrances to the circles."
benchList.append(Benchmark("Entrances", CollisionChecker(
    trapField), start, goal, description, 2))

# -----------------------------------------
scene = dict()
scene['K'] = Polygon([(0, 8), (0, 17), (3, 17), (3, 13.5), (5.5, 17),
                     (9, 17), (5.5, 12.5), (9, 8), (5.5, 8), (3, 11.5), (3, 8)])
scene['I'] = Polygon([(9.5, 8), (9.5, 17), (12.5, 17), (12.5, 8)])
scene['T'] = Polygon([(16, 8), (16, 14), (13, 14), (13, 17),
                     (22, 17), (22, 14), (19, 14), (19, 8)])

description = 'Not an really serious challenge.'
benchList.append(Benchmark('KIT', CollisionChecker(
    scene), [[1, 1]], [[21, 21]], description, 1))

# -----------------------------------------
scene = dict()
line = LineString([(0, 0), (0, 22), (22, 22), (22, 0), (0, 0)]).buffer(2.0)
scene['rim'] = line.buffer(-1.0)
line = LineString([(6, 6), (6, 16), (16, 16), (16, 6), (6, 6)]).buffer(0.75)
scene['obs'] = line.buffer(-0.25)

description = 'A rounded rectangle inside a rounded rectangle..'
benchList.append(Benchmark('Inside', CollisionChecker(
    scene), [[3, 3]], [[19, 19]], description, 1))

# -----------------------------------------
scene = dict()
scene['obs0'] = LineString([(0, 11.5), (6.33, 11.5)]).buffer(1.0)
scene['obs1'] = LineString([(22, 11.5), (15.66, 11.5)]).buffer(1.0)
scene['obs2'] = LineString([(11, 5), (11, 17)]).buffer(1.0)
scene['obs3'] = LineString([(5.5, 0), (5.5, 6)]).buffer(1.0)
scene['obs4'] = LineString([(16.5, 0), (16.5, 6)]).buffer(1.0)
scene['obs5'] = LineString([(5.5, 22), (5.5, 17)]).buffer(1.0)
scene['obs6'] = LineString([(16.5, 22), (16.5, 17)]).buffer(1.0)

description = 'A simple smetic labyrinth.'
benchList.append(Benchmark('SSL', CollisionChecker(
    scene), [[1, 1]], [[21, 21]], description, 2))

# -----------------------------------------
scene = dict()
a = Point(11, 11).buffer(8)
b = Point(11, 11).buffer(7.5)
scene['obs'] = a.difference(b)

description = 'A simple ring.'
benchList.append(Benchmark('Ring', CollisionChecker(
    scene), [[4, 4]], [[18, 18]], description, 2))

# -----------------------------------------
scene = dict()
a = Point(5.25, 5.25).buffer(5.5)
b = Point(4.75, 5.25).buffer(5.5)
scene['obs0'] = a.difference(b)
a = Point(5.25, 16.75).buffer(5.5)
b = Point(4.75, 16.75).buffer(5.5)
scene['obs1'] = a.difference(b)
a = Point(16.75, 5.25).buffer(5.5)
b = Point(17.25, 5.25).buffer(5.5)
scene['obs2'] = a.difference(b)
a = Point(16.75, 16.75).buffer(5.5)
b = Point(17.25, 16.75).buffer(5.5)
scene['obs3'] = a.difference(b)
a = Point(9, 11).buffer(5.5)
b = Point(9.5, 11).buffer(5.5)
scene['obs4'] = a.difference(b)
a = Point(13, 11).buffer(5.5)
b = Point(12.5, 11).buffer(5.5)
scene['obs5'] = a.difference(b)

description = 'Six Hemispheres have to be mastered.'
benchList.append(Benchmark('Hemispheres', CollisionChecker(
    scene), [[1, 1]], [[21, 21]], description, 3))

# -----------------------------------------
simpleField = dict()
simpleField["obs1"] = LineString([(12.5, 0), (12.5, 15)]).buffer(1.0)
simpleField["obs2"] = LineString([(7, 15), (18, 15)]).buffer(1.0)
simpleDescription = "Around the hammerhead"
benchList.append(Benchmark("Hammerhead", CollisionChecker(
    simpleField), [[10, 1]], [[15, 1]], simpleDescription, 1))


# -----------------------------------------
mediumField = dict()
mediumField["obs1"] = LineString([(0, 20), (15, 20)]).buffer(1.0)
mediumField["obs2"] = LineString([(0, 10), (15, 10)]).buffer(1.0)
mediumField["obs3"] = LineString([(10, 15), (25, 15)]).buffer(1.0)
mediumField["obs4"] = LineString([(10, 5), (25, 5)]).buffer(1.0)
mediumDescription = "Through the zigzag"
benchList.append(Benchmark("Zigzag", CollisionChecker(mediumField), [
                 [12.5, 24]], [[12.5, 1]], mediumDescription, 2))


# -----------------------------------------
# Compute spiral points to add to line string
def spiralPoints(center=(12.5, 12.5), radius=10, numPoints=30, coils=4):
    points = []
    awayStep = float(radius)/float(numPoints)
    aroundStep = float(coils)/float(numPoints)
    aroundRadians = aroundStep * 2 * math.pi
    rotation = math.pi
    for i in range(numPoints):
        away = i * awayStep
        around = i * aroundRadians + rotation
        x = center[0] + math.cos(around) * away
        y = center[1] + math.sin(around) * away
        points.append((x, y))
    return points


hardField = dict()
hardField["obs1"] = LineString(spiralPoints(
    center=(12.5, 12.5), radius=10, numPoints=300, coils=4)).buffer(0.1)
hardDescription = "Through the spiral"
benchList.append(Benchmark("Spiral", CollisionChecker(hardField), [
                 [12.5, 24]], [[13.1, 12.9]], hardDescription, 3))


# -----------------------------------------
scene = dict()
scene["obs1"] = LineString([(20, 25), (20, 20), (24, 20)]).buffer(0.1)
scene["obs2"] = LineString([(25, 19), (19, 19), (19, 24)]).buffer(0.1)
scene["obs3"] = LineString([(1, 5), (5, 5), (5, 0)]).buffer(0.1)
scene["obs4"] = LineString([(6, 1), (6, 6), (0, 6)]).buffer(0.1)
scene["obs5"] = LineString([(19, 24), (5, 7)]).buffer(0.1)
scene["obs6"] = LineString([(1, 15), (20, 15)]).buffer(0.1)
description = 'Medium challenge. Large free space with narrow passings'
benchList.append(Benchmark('medium', CollisionChecker(
    scene), [[1, 1]], [[24, 24]], description, 1))

# -----------------------------------------
mediumField = dict()
pol = Polygon([(17, 0), (17, 25), (25, 25), (25, 0)]).buffer(1.0)
line = LineString([(17.5, 22.5), (22.5, 17.5), (17.5, 12.5),
                  (22.5, 12.5), (17.5, 7.5), (22.5, 2.5)]).buffer(0.75)
mediumField["obs1"] = pol.difference(line)

description = 'Medium challenge. Large free space with only one narrow way '
benchList.append(Benchmark('medium2', CollisionChecker(mediumField), [
                 [17.5, 22.5]], [[22.5, 2.5]], description, 1))

# -----------------------------------------
scene = dict()

scene["top"] = Polygon([[5, 25], [5, 10], [10, 10], [
    10, 20], [15, 20], [15, 25]])
scene["bottom"] = Polygon(
    [[10, 0], [10, 5], [15, 5], [15, 15], [20, 15], [20, 0]])

description = 'Easy challenge'
benchList.append(Benchmark('L-square', CollisionChecker(scene),
                 [[5, 5]], [[20, 20]], description, 1))

# -----------------------------------------
scene = dict()

scene["teeth_bottom"] = Polygon(
    [(7.5, 0), (10, 15), (12.5, 0), (15, 15), (17.5, 0)])
scene["tooth_top"] = Polygon([(10, 25), (12.5, 10), (15, 25)])

description = 'Medium challenge'
benchList.append(Benchmark('Teeth', CollisionChecker(
    scene), [[2, 2]], [[23, 2]], description, 2))

# -----------------------------------------
scene = dict()

gearshift_polygon = []
gearshift_x = 2
gearshift_y = 3
slot_count = 10
slot_width = 1
slot_length = 8
slot_spacing = 1
gearshift_width = slot_spacing + slot_count * (slot_width + slot_spacing)
gearshift_heigth = 2 * slot_spacing + 2 * slot_length + slot_width

gearshift_polygon.append((gearshift_x, gearshift_y))
gearshift_polygon.append((gearshift_x + gearshift_width, gearshift_y))
gearshift_polygon.append(
    (gearshift_x + gearshift_width, gearshift_y + gearshift_heigth))
gearshift_polygon.append((gearshift_x, gearshift_y + gearshift_heigth))
gearshift_polygon.append(
    (gearshift_x, gearshift_y + slot_spacing + slot_length + slot_width))

# upper slots
for i in range(0, slot_count):
    slot_start_x = gearshift_x + slot_spacing + i * (slot_width + slot_spacing)
    slot_start_y = gearshift_y + slot_spacing + slot_length + slot_width
    gearshift_polygon.append((slot_start_x, slot_start_y))
    gearshift_polygon.append((slot_start_x, slot_start_y + slot_length))
    gearshift_polygon.append(
        (slot_start_x + slot_width, slot_start_y + slot_length))
    gearshift_polygon.append((slot_start_x + slot_width, slot_start_y))

# lower slots
for i in range(slot_count - 1, -1, -1):
    slot_start_x = gearshift_x + slot_spacing + i * \
        (slot_width + slot_spacing) + slot_width
    slot_start_y = gearshift_y + slot_spacing + slot_length
    gearshift_polygon.append((slot_start_x, slot_start_y))
    gearshift_polygon.append((slot_start_x, slot_start_y - slot_length))
    gearshift_polygon.append(
        (slot_start_x - slot_width, slot_start_y - slot_length))
    gearshift_polygon.append((slot_start_x - slot_width, slot_start_y))

gearshift_polygon.append(
    (gearshift_x, gearshift_y + slot_spacing + slot_length))

scene["gearshift"] = Polygon(gearshift_polygon)

description = 'Hard challenge'
benchList.append(Benchmark('Gearshift', CollisionChecker(
    scene), [[24, 2]], [[21.5, 4.5]], description, 3))

# -----------------------------------------


def rectangle(x, y, width, height=None):
    if height is None:
        height = width
    return Polygon([(x, y+height), (x+width, y+height), (x + width, y), (x, y)])


def zigzag(x, y, length):
    baseX = x
    baseY = y
    x_coords = [baseX + i for i in range(length)]
    y_coords = [baseY + (i % 2) for i in range(length)]
    return LineString(zip(x_coords, y_coords)).buffer(.2)


benchmark_easy = dict()
benchmark_easy["obs1"] = rectangle(5, 10, 3)
benchmark_easy["obs2"] = rectangle(10, 15, 3)
benchmark_easy["obs3"] = rectangle(10, 5, 3)
benchmark_easy["obs4"] = rectangle(15, 10, 3)
description = "Four squares to avoid"
# limits = [[0,0],[23,23]]
start = [11.5, 22]
goal = [11.5, 2]
difficulty = 1
benchList.append(Benchmark("Squares_easy", CollisionChecker(
    benchmark_easy), [start], [goal], description, difficulty))

benchmark_medium = dict()
benchmark_medium["obs1"] = zigzag(5, 18, 15)
benchmark_medium["obs2"] = zigzag(0, 12, 10)
benchmark_medium["obs3"] = zigzag(15, 12, 10)
benchmark_medium["obs4"] = zigzag(5, 5, 15)
description = "Four zigzag lines to avoid"
# limits = [[0,0],[23,23]]
start = [11.5, 21]
goal = [11.5, 2]
difficulty = 2
benchList.append(Benchmark("Zigzag_medium", CollisionChecker(
    benchmark_medium), [start], [goal], description, difficulty))

benchmark_hard = dict()
for i in range(0, 22, 8):
    benchmark_hard["row_a" + str(i)] = zigzag(0, i + 4, 18)
    benchmark_hard["row_b" + str(i)] = zigzag(8, i, 18)
# limits = [[0,0],[23,23]]
description = "Many zigzag lines to avoid"
start = [22, 22]
goal = [1, 0.8]
difficulty = 3
benchList.append(Benchmark("Zigzag_hard", CollisionChecker(
    benchmark_hard), [start], [goal], description, difficulty))

# -----------------------------------------
japanField = dict()
japanField["obs1"] = Point([12, 12]).buffer(5.0)
description = "Easy field with circle"
benchList.append(Benchmark("Japan", CollisionChecker(
    japanField), [[18, 13]], [[6, 10]], description, 2))


# -----------------------------------------
haystackField = dict()
middleCircle = [12, 12]
for i in range(9):
    haystackField["obs1"+str(i)] = LineString([(21-2*i, 21),
                                               middleCircle]).buffer(0.05)
    haystackField["obs2"+str(i)] = LineString([(1+2*i, 2),
                                               middleCircle]).buffer(0.05)
    haystackField["obs3"+str(i)] = LineString([(2, 21-2*i),
                                               middleCircle]).buffer(0.05)
    haystackField["obs4"+str(i)] = LineString([(21, 1+2*i),
                                               middleCircle]).buffer(0.05)
description = "Needle in a haystack"
benchList.append(Benchmark("Haystack", CollisionChecker(
    haystackField), [[5, 10.5]], [[19, 11.7]], description, 3))

# -----------------------------------------
hairPerson = dict()
middleCircle = [12, 12]
hairPerson["obs1"] = Point(middleCircle).buffer(5.0)
hairPerson["obs2"] = Polygon([(7, 0), (7, 6), (17, 6), (17, 0)]).buffer(1.0)
for i in range(10):
    hairPerson["obs3"+str(i)] = LineString([(21-2*i, 21),
                                            middleCircle]).buffer(0.2)
description = "From Shoulder to shoulder"
benchList.append(Benchmark("HairPerson", CollisionChecker(
    hairPerson), [[9, 7.5]], [[15, 7.5]], description, 4))

# ------------------------------------------
# ------------------------------------------
# ------------------------------------------
# ------------------------------------------
# ------------------------------------------
# Ab hier Bechmarks aus unserer Vorlesung
# ------------------------------------------

mShapeField = dict()
mShapeField["obs1"] = LineString(
    [(4, 8), (4, 18), (12, 18), (20, 18), (20, 8)]).buffer(1.0)
mShapeField["obs2"] = LineString([(12, 8), (12, 18)]).buffer(1.0)
description = "Planer has to find a path between two points placed in an M shaped obstacle"
benchList.append(Benchmark("M Sahpe(JM)", CollisionChecker(
    mShapeField), [[7, 15]], [[17, 15]], description, 2))
# -----------------------------------------

randomField = dict()
randomField["point1"] = Point([10, 5]).buffer(1.0).buffer(3.0)
randomField["point2"] = Point([19, 18]).buffer(1.0).buffer(2.0)
randomField["obs1"] = Polygon([(8, 14), (13, 14), (13, 11), (6, 11)])
randomField["obs2"] = Polygon([(16, 0), (19, 0), (19, 12), (16, 12)])
randomField["obs3"] = LineString(
    [(3, 10), (4, 18), (12, 16), (14, 19)]).buffer(1.0)
randomField["obs4"] = LineString([[1, 4], [7, 9]]).buffer(.2)
description = "Planer has to find a path in a field of random objects."
benchList.append(Benchmark("Random Field (LB)", CollisionChecker(
    randomField), [[5, 21]], [[21, 1]], description, 2))
# -----------------------------------------

stickfigure = dict()
stickfigure["obs1"] = LineString([(12, 15), (12, 5)]).buffer(1.0)
stickfigure["obs2"] = LineString([(7, 18), (12, 12)]).buffer(1.0)
stickfigure["obs3"] = LineString([(12, 18), (12, 12)]).buffer(1.0)
stickfigure["obs4"] = LineString([(16, 18), (12, 12)]).buffer(1.0)
stickfigure["obs5"] = Point([12, 20]).buffer(2.5)
stickfigure["obs6"] = LineString([(12, 5), (8, 2)]).buffer(1.0)
stickfigure["obs7"] = LineString([(12, 5), (16, 2)]).buffer(1.0)


description = "Planer has to find a passage around stick figure"
benchList.append(Benchmark("stick figure (AA)", CollisionChecker(
    stickfigure), [[1, 2]], [[18, 6]], description, 2))


# -----------------------------------------
DCircleShapeField = dict()
DCircleShapeField["obs1"] = Point(7.5, 11).buffer(
    7.0)-Point(7.5, 11).buffer(6.5)-LineString([(9, 11), (16, 11)]).buffer(.5)
DCircleShapeField["obs2"] = Point(16, 11).buffer(
    7.0)-Point(16, 11).buffer(6.5)-LineString([(9, 11), (16, 11)]).buffer(.5)
description = "Planer has to find a narrow passage within two circles"
benchList.append(Benchmark("Double Circle(JM)", CollisionChecker(
    DCircleShapeField), [[5, 11]], [[17, 11]], description, 3))


# -----------------------------------------
sunScribble = dict()
sunScribble["scribble"] = LineString(
    [(8, 3), (3, 6), (6, 8), (16, 8), (10, 18), (18, 18)]).buffer(0.5)
sunScribble["sun_base"] = Point([0, 22]).buffer(4.0)
sunScribble["sun_ray1"] = LineString([(1, 18), (1, 15)]).buffer(0.2)
sunScribble["sun_ray2"] = LineString([(3.33, 19.4), (6.8, 16.8)]).buffer(0.2)
sunScribble["sun_ray3"] = LineString([(4, 22), (8, 22)]).buffer(0.2)
description = "Planer has to find a path in a childrens picture with narrow passages."
benchList.append(Benchmark("Sun Scribble (FP)", CollisionChecker(
    sunScribble), [[2, 8]], [[17, 15]], description, 3))
# -----------------------------------------
Wpoint = dict()

Wpoint["ob1"] = LineString([[5, 15], [10, 5]]).buffer(1.0)
Wpoint["ob2"] = LineString([[10, 5], [12, 8]]).buffer(1.0)
Wpoint["ob3"] = LineString([[12, 8], [14, 5]]).buffer(1.0)
Wpoint["ob4"] = LineString([[14, 5], [19, 15]]).buffer(1.0)
Wpoint["point"] = Point([12, 15]).buffer(0.5).buffer(1.0)

description = "Planer has to find a passage around a W with a point."
benchList.append(Benchmark("Wpoint (SS)", CollisionChecker(
    Wpoint), [[12, 3]], [[8, 14]], description, 2))
# -----------------------------------------
pointsField = dict()
pointsField["obj1"] = LineString(
    [(0, 18), (5, 10), (0, 2), (15, 10), (0, 18)]).buffer(0.7)
pointsField["obj2"] = Point([20, 10]).buffer(1.0).buffer(2)
pointsField["obj3"] = Point([10, 20]).buffer(1.0).buffer(2)
pointsField["obj4"] = Point([10, 1]).buffer(1.0).buffer(2)
pointsField["obj5"] = Point([20, 20]).buffer(1.0).buffer(2)
pointsField["obj6"] = Point([20, 1]).buffer(1.0).buffer(2)
description = "Planer has to find a narrow passage between several points"
benchList.append(Benchmark("Points Field (NC)", CollisionChecker(
    pointsField), [[3, 1]], [[5, 20]], description, 2))
# -----------------------------------------
Hi = dict()
Hi["H"] = LineString([(8, 18), (6, 18), (6, 10), (6, 2), (8, 2), (6, 2), (6, 10),
                     (16, 10), (16, 18), (14, 18), (16, 18), (16, 2), (14, 2)]).buffer(1.0)
Hi["i"] = LineString([(20, 2), (20, 5)]).buffer(1.0)
Hi["i2"] = Point([20, 8]).buffer(0.5).buffer(0.5)
description = "Planer has to find a passage through two narrow passages"
benchList.append(Benchmark("Hi (TN)", CollisionChecker(Hi),
                 [[11, 7.5]], [[11, 12.5]], description, 3))
# -----------------------------------------
Field_TEST = dict()
Field_TEST["obs1"] = LineString(
    [(18, 18), (6, 18), (6, 10), (18, 10), (18, 3), (6, 3)]).buffer(1.0)
Field_TEST["obs2"] = LineString(
    [(2, 0), (2, 22), (22, 22), (22, 0)]).buffer(0.5)
description = "Planer has to find a way from one notch to the other"
benchList.append(Benchmark("S Field", CollisionChecker(
    Field_TEST), [[15, 6]], [[8, 15]], description, 2))
# -----------------------------------------
HKA = dict()

HKA["H1"] = LineString([[6, 2.5], [6, 7.5]]).buffer(0.5)
HKA["H2"] = LineString([[3.5, 5], [8.5, 5]]).buffer(0.5)
HKA["H3"] = LineString([[10, 2.5], [10, 7.5]]).buffer(0.5)
HKA["K4"] = LineString([[12, 2.5], [12, 7.5]]).buffer(0.5)
HKA["K5"] = LineString([[12, 5], [14.5, 7.5]]).buffer(0.5)
HKA["K6"] = LineString([[12, 5], [14.5, 2.5]]).buffer(0.5)
HKA["A7"] = LineString([[16, 2.5], [18.5, 7.5]]).buffer(0.5)
HKA["A8"] = LineString([[18.5, 7.5], [21, 2.5]]).buffer(0.5)
HKA["A9"] = LineString([[17, 4], [20, 4]]).buffer(0.5)


description = "Planer has to find a passage around HKA icon."
benchList.append(Benchmark("HKA (SS)", CollisionChecker(HKA),
                 [[4, 3]], [[13, 7]], description, 3))
# -----------------------------------------
HKA = dict()
HKA["obs1"] = LineString(
    [(3, 16), (3, 6), (3, 11), (1.5, 11), (5.5, 11)]).buffer(0.5)
HKA["obs2"] = LineString([(9.5, 16), (9.5, 6), (9.5, 10),
                         (13, 16), (9.5, 10), (9.5, 12), (13, 6)]).buffer(0.5)
HKA["obs3"] = LineString(
    [(15, 6), (17, 16), (19, 6), (18.4, 9), (16, 9)]).buffer(0.5)
HKA["obs4"] = LineString([(7, 11), (7, 16), (7, 6)]).buffer(0.5)

description = "Planer has to find a path in the Icon of the Hochschule Karlsruhe"
benchList.append(Benchmark("HKA (HKA)", CollisionChecker(HKA),
                 [[5, 8]], [[5, 14]], description, 2))

# -----------------------------------------
"""
tLines = dict()
tLines["part1"] = LineString([(5, 15), (20, 30)]).buffer(0.5)
tLines["part2"] = LineString([(30, 20), (20, 30)]).buffer(0.5)
tLines["part3"] = LineString([(25, 10), (5, 30)]).buffer(0.5)
tLines["part4"] = LineString([(25, 10), (5, 30)]).buffer(0.5)
tLines["part5"] = LineString([(10, 15), (20, 25)]).buffer(0.5)
tLines["part6"] = LineString([(3, 10), (25, 10)]).buffer(0.5)
description = "Planer has to find a passage around the lines."
benchList.append(Benchmark("tLines (TK)", CollisionChecker(
    tLines), [[15, 22]], [[13, 20]], description, 2))
"""
# -----------------------------------------
uLines = dict()
uLines["obs1"] = LineString([(5, 18), (5, 2), (10, 2), (11, 14)]).buffer(0.5)
uLines["obs2"] = LineString([(7, 10), (7, 19), (17, 19), (17, 10)]).buffer(1.0)
uLines["obs3"] = Polygon([(15, 5), (20, 3), (18, 1), (12, 2)])
uLines["obs4"] = Point([2, 12]).buffer(0.25)
description = "Planer has to find a path between U-shaped lines"
benchList.append(Benchmark("U Lines (KA)", CollisionChecker(
    uLines), [[3, 10]], [[8, 14]], description, 2))
# -----------------------------------------

points = np.linspace(0, 20, num=60)


def wellen(points):
    P = list()
    for i in points:
        P.append((i, math.sin(i)+7))
    return P


rico2 = dict()
rico2["obs1"] = Polygon([[5, 15], [5, 10], [10, 10]])
rico2["obs2"] = LineString(wellen(points)).buffer(0.5)
rico2["obs3"] = Point(17, 17).buffer(6).difference(
    LineString([(20, 10), (12, 20)]).buffer(1))
rico2["obs4"] = Polygon([[5, 0], [20, 0], [15, 3]])
rico2["obs5"] = Point(3, 20).buffer(3).difference(Point(3, 20).buffer(1))
description = "Ricos Benchmark mit engen Durchgängen"
benchList.append(Benchmark("Ricos Moderne Kunst", CollisionChecker(
    rico2), [[0, 0]], [[0, 10]], description, 3))

# -----------------------------------------
Pillars = dict()
Pillars["obs1"] = Polygon([(2, 0), (4, 0), (4, 21), (2, 21)])
Pillars["obs2"] = Polygon([(6, 2), (8, 2), (8, 23), (6, 23)])
Pillars["obs3"] = Polygon([(10, 0), (12, 0), (12, 21), (10, 21)])
Pillars["obs4"] = Polygon([(14, 2), (16, 2), (16, 23), (14, 23)])
Pillars["obs5"] = Polygon([(18, 0), (20, 0), (20, 21), (18, 21)])
description = "Planer has to find a passage between the pillars."
benchList.append(Benchmark("Pillars (MA)", CollisionChecker(
    Pillars), [[1, 1]], [[22, 1]], description, 3))
# -----------------------------------------
bubbleScene = dict()
nrOfPoints = 170
for i in range(nrOfPoints):
    name = "point_" + str(i)
centerPoint = [random.uniform(0, 20), random.uniform(0, 20)]
bubbleScene[name] = Point(centerPoint).buffer(0.25).buffer(0.25)
pass
description = "Planner has to find path between random generated bubbles"
benchList.append(Benchmark("Random Bubbles (MS)", CollisionChecker(
    bubbleScene), [[0, 0]], [[21, 21]], description, 3))
# -----------------------------------------
"""
scene = dict()

x = 5
y = 5

for ind_x in range(0, 4):
    x = 5
    for ind_y in range(0, 4):

        scene[str(ind_x) + str(ind_y)] = Point([x, y]
                                               ).buffer(2.0 + ind_x/3 + ind_y/3)

        x += 5 + 2*ind_y + ind_x * 3
        y += 5 + ind_x * 4
    
description = "Point Matrix."
benchList.append(Benchmark("Point Matrix (CH)", CollisionChecker(scene, limits=[
                 [0.0, 40.0], [0.0, 40.0]]), [[30, 35]], [[7.5, 7.5]], description, 2))
"""
# -----------------------------------------
vfb = dict()
vfb["obs1"] = LineString([(2, 15), (4, 6), (6, 15)]).buffer(1.0)
vfb["obs2"] = LineString([(10, 6), (10, 15), (15, 15)]).buffer(1.0)
vfb["obs3"] = LineString([(10, 12), (15, 12)]).buffer(1.0)
vfb["obs4"] = LineString([(18, 6), (18, 15), (22, 13),
                         (22, 12), (18, 10), (22, 9), (22, 8), (18, 6)]).buffer(1.0)
description = "Planer has to find a narrow passage with a significant extend."
benchList.append(Benchmark("VFB (LD)", CollisionChecker(vfb),
                 [[2, 2]], [[19, 19]], description, 1))
# -----------------------------------------
face = dict()
face["lefteye"] = Point([7, 18]).buffer(1.0).buffer(1.0)
face["righteye"] = Point([17, 18]).buffer(1.0).buffer(1.0)
face["nose"] = LineString([[12, 15], [8, 10]]).buffer(1.0)
face["nose2"] = LineString([[8, 10], [14, 10]]).buffer(1.0)
face["mouth"] = LineString([[5, 8], [9, 4]]).buffer(1.0)
face["mouth2"] = LineString([[9, 4], [15, 4]]).buffer(1.0)
face["mouth3"] = LineString([[15, 4], [19, 8]]).buffer(1.0)
face["eyebrow1"] = LineString([[2, 18], [7, 22]]).buffer(1.0)
face["eyebrow2"] = LineString([[22, 18], [17, 22]]).buffer(1.0)

benchList.append(Benchmark("Face (JS)", CollisionChecker(
    face), [[1, 20]], [[13, 12, 5]], description, 2))
# -----------------------------------------
maze = dict()
for i in range(1, 5):
    maze["line1"+str(i)] = LineString([(10-i*2, 10-i*2),
                                       (10-i*2, 10+i*2)]).buffer(0.3)
    maze["line2"+str(i)] = LineString([(10-i*2, 10-i*2),
                                       (10+i*2, 10-i*2)]).buffer(0.3)
    maze["line3"+str(i)] = LineString([(10+i*2, 10-i*2),
                                       (10+i*2, 10+i*2)]).buffer(0.3)
    maze["line4"+str(i)] = LineString([(10-i*2, 10+i*2),
                                       (8+i*2, 10+i*2), (8+i*2, 8+i*2)]).buffer(0.3)
description = "maze of death with 5 walls"
benchList.append(Benchmark("Maze (AZ)", CollisionChecker(
    maze), [[9, 11]], [[9, 20]], description, 4))
# -----------------------------------------
robo = dict()
robo["base"] = LineString([(5, 0), (5, 1)]).buffer(3)
robo["arm1"] = LineString([(5, 3), (8, 8)]).buffer(1)
robo["elbow"] = Point([9, 9]).buffer(1.5)
robo["arm2"] = LineString([(9.5, 9.5), (14, 10)]).buffer(1)
robo["wrist"] = Point([15, 10]).buffer(1.5)
robo["arm3"] = LineString([(16, 9.5), (18, 7)]).buffer(1)
description = "Little 3 joint robot."
benchList.append(Benchmark("Robo (FP)", CollisionChecker(
    robo), [[2, 8]], [[15, 5]], description, 2))
# -----------------------------------------
stickman = dict()
stickman["head"] = Point([8.5, 12.5]).buffer(2.0)
stickman["body"] = LineString([(8.5, 10), (8.5, 5)]).buffer(0.5)
stickman["left arm"] = LineString([(8, 8.5), (4, 12)]).buffer(0.5)
stickman["right arm"] = LineString([(9, 8.5), (13, 12)]).buffer(0.5)
stickman["left foot"] = LineString([(8, 5), (6, 1.5)]).buffer(0.5)
stickman["right foot"] = LineString([(9, 5), (11, 1.5)]).buffer(0.5)

description = "Planer has to find a path in a field of random objects."
benchList.append(Benchmark("Stickman (MK)", CollisionChecker(
    stickman), [[8.5, 3.5]], [[9.5, 10.25]], description, 2))
# -----------------------------------------
house = dict()
house["line1"] = LineString([(4, 6), (4, 11)]).buffer(0.7)
house["line2"] = LineString([(6, 13), (14, 13)]).buffer(0.7)
house["line3"] = LineString([(6, 4), (14, 4)]).buffer(0.7)
house["line4"] = LineString([(16, 6), (16, 11)]).buffer(0.7)
house["line5"] = LineString([(4, 15), (9, 20)]).buffer(0.7)
house["line6"] = LineString([(16, 15), (11, 20)]).buffer(0.7)
description = "House with broken walls"
benchList.append(Benchmark("House (AZ)", CollisionChecker(
    house), [[10, 17]], [[10, 7]], description, 2))
# -----------------------------------------
zwanzigdreisigplus = dict()
# 2030+
zwanzigdreisigplus["obs1"] = LineString(
    [(1, 17), (3, 17), (3, 12), (1, 12), (1, 7), (3, 7)]).buffer(0.5)
zwanzigdreisigplus["obs2"] = LineString(
    [(5, 17), (7, 17), (7, 7), (5, 7), (5, 17)]).buffer(0.5)
zwanzigdreisigplus["obs3"] = LineString(
    [(9, 17), (11, 17), (11, 12), (9, 12), (11, 12), (11, 7), (9, 7)]).buffer(0.5)
zwanzigdreisigplus["obs4"] = LineString(
    [(13, 17), (15, 17), (15, 7), (13, 7), (13, 17)]).buffer(0.5)
zwanzigdreisigplus["obs5"] = LineString(
    [(17, 12), (22, 12), (19.5, 12), (19.5, 15), (19.5, 9)]).buffer(0.5)
# LOL
zwanzigdreisigplus["obs6"] = LineString(
    [(15, 5), (15, 2), (16, 2)]).buffer(0.2)
zwanzigdreisigplus["obs7"] = LineString(
    [(17, 5), (17, 2), (18, 2), (18, 5), (17, 5)]).buffer(0.2)
zwanzigdreisigplus["obs8"] = LineString(
    [(19, 5), (19, 2), (20, 2)]).buffer(0.2)

description = "Planer has to find a path in the Icon of the HKA 2030+"
benchList.append(Benchmark("2030+ (2030+)", CollisionChecker(
    zwanzigdreisigplus), [[3, 10]], [[20, 3]], description, 2))
# -----------------------------------------


def checkBubblePoint(bubblePoint, testPoint, bubbleRadius):
    bP = bubblePoint
    bR = bubbleRadius * 10
    tP = testPoint
    if (((tP[0] >= (bP[0] - bR)) & (tP[0] <= (bP[0] + bR))) & ((tP[1] >= (bP[1] - bR)) & (tP[1] <= (bP[1] + bR)))):
        clear = False
    else:
        clear = True
        pass
    return clear


bubbleScene = dict()
nrOfPoints = 400
startPoint = [0, 0]
targetPoint = [20, 20]
buffer = 0.2

for i in range(nrOfPoints):
    name = "point_" + str(i)
    centerPoint = [random.uniform(0, 20), random.uniform(0, 20)]
    if (checkBubblePoint(centerPoint, startPoint, buffer) & checkBubblePoint(centerPoint, targetPoint, buffer)):
        bubbleScene[name] = Point(centerPoint).buffer(buffer).buffer(buffer)
        pass
    pass

description = "Planner has to find path between random generated bubbles"
benchList.append(Benchmark("Random Bubbles (MS)", CollisionChecker(
    bubbleScene), [[0, 0]], [[21, 21]], description, 3))
