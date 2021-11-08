import numpy as np
import math
from fractions import Fraction
from scipy.odr import *


class featureDetection:
    def __init__(self):
        self.EPSILON = 10
        self.DELTA = 20
        self.SNUM = 6
        self.PMIN = 20
        self.GMAX = 20
        self.SEED_SEGMENT = []
        self.LINE_SEGMENT = []
        self.LASERPOINTS = []
        self.LINE_PARAMS = None
        self.NP = len(self.LASERPOINTS) - 1
        self.LMIN = 20  # minimum length of a line
        self.LR = 0  # real length of a line
        self.PR = 0  # real number of laser points in a line segment

    def dist_point2point(self, point1, point2):
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    def dist_pt2ln(self, params, point):
        a, b, c = params
        distance = abs(a * point[0] + b * point[1] + c) / math.sqrt(a ** 2 + b ** 2)
        return distance

    def line_2pts(self, m, b):
        x = 5
        y = m * x + b
        x2 = 2000
        y2 = m * x2 + b
        return [(x, y), (x2, y2)]

    def lineForm_G2SI(self, a, b, c):
        m = -a / b
        b = -c / b
        return m, b

    def lineForm_Si2G(self, m, b):
        a,b,c = -m, 1, -b
        if a < 0:
            a, b, c = -a, -b, -c
        den_a = Fraction(a).limit_denominator(1000).as_integer_ratio()[1]
        den_c = Fraction(c).limit_denominator(1000).as_integer_ratio()[1]

        gcd = np.gcd(den_a, den_c)
        lcm = den_a * den_c / gcd

        a = a * lcm
        b = b * lcm
        c = c * lcm
        return a,b,c

    def line_intercept_general(self, param1, param2):
        a1, b1, c1 = param1
        a2, b2, c2 = param2
        x = (c1 * b2 - b1 * c2) / (b1 * a2 - a1 * b2)
        y = (a1 * c2 - a2 * c1) / (b1 * a2 - a1 * b2)
        return x, y

    def points2line(self, point1, point2):
        m, b = 0, 0
        if point2[0] == point1[0]:
            pass
        else:
            m = (point2[1] - point1[1]) / (point2[0] - point1[0])
            b = point2[1] - m * point2[0]
        return m, b

    def projection_pt2ln(self, point, m, b):
        x, y = point
        m2 = -1 / m
        c2 = y - m2 * x
        intersection_x = - (b - c2) / (m - m2)
        intersection_y = m2 * intersection_x + c2
        return intersection_x, intersection_y

    def AD2pos(self, distance, theta, robotposition):
        x = distance * math.cos(theta) + robotposition[0]
        y = -distance * math.sin(theta) + robotposition[1]
        return (int(x), int(y))

    def laser_point_set(self, data):
        self.LASERPOINTS = []
        if not data:
            pass
        else:
            for point in data:
                coordinates = self.AD2pos(point[0], point[1], point[2])
                self.LASERPOINTS.append([coordinates, point[1]])

        self.NP = len(self.LASERPOINTS) - 1

    def linear_func(self, p, x):
        m, b = p
        return m * x + b

    def odr_fit(self, laser_points):
        x = np.array([i[0][0] for i in laser_points])
        y = np.array([i[0][1] for i in laser_points])

        linear_model = Model(self.linear_func)
        data = RealData(x, y)
        odr_model = ODR(data, linear_model, beta0=[0., 0.])

        out = odr_model.run()
        m, b = out.beta
        return m, b

    def predictPoint(self, line_params, sensed_point, robotpos):
        m, b = self.points2line(robotpos, sensed_point)
        param1 = self.lineForm_Si2G(m, b)
        predx, predy = self.line_intercept_general(param1, line_params)
        return predx, predy

    def seed_segment_detection(self, robot_position, break_point_ind):
        flag = True
        self.NP = max(0, self.NP)
        self.SEED_SEGMENT = []
        for i in range(break_point_ind, (self.NP - self.PMIN)):
            predicted_points_to_draw = []
            j = i + self.SNUM
            m, c = self.odr_fit(self.LASERPOINTS[i:j])

            params = self.lineForm_Si2G(m, c)

            for k in range(i, j):
                predicted_point = self.predictPoint(params, self.LASERPOINTS[k][0], robot_position)
                predicted_points_to_draw.append(predicted_point)
                d1 = self.dist_point2point(predicted_point, self.LASERPOINTS[k][0])

                if d1 > self.DELTA:
                    flag = False
                    break

                d2 = self.dist_pt2ln(params, self.LASERPOINTS[k][0])

                if d2 > self.EPSILON:
                    flag = False
                    break

            if flag:
                self.LINE_PARAMS = params
                return [self.LASERPOINTS[i:j], predicted_points_to_draw, (i, j)]

        return False

    def seed_segment_grow(self, indicies, break_point):
        line_eq = self.LINE_PARAMS
        i, j = indicies
        PB, PF = max(break_point, i - 1), min(j + 1, len(self.LASERPOINTS) - 1)

        while self.dist_pt2ln(line_eq, self.LASERPOINTS[PF][0]) < self.EPSILON:
            if PF > self.NP - 1:
                break
            else:
                m, b = self.odr_fit(self.LASERPOINTS[PB:PF])
                line_eq = self.lineForm_Si2G(m, b)

                POINT = self.LASERPOINTS[PF][0]

            PF = PF + 1
            NEXTPOINT = self.LASERPOINTS[PF][0]

            if self.dist_point2point(POINT, NEXTPOINT) > self.GMAX:
                break

        PF = PF - 1

        while self.dist_pt2ln(line_eq, self.LASERPOINTS[PB][0]) < self.EPSILON:
            if PB < break_point:
                break
            else:
                m, b = self.odr_fit(self.LASERPOINTS[PB:PF])
                line_eq = self.lineForm_Si2G(m, b)
                POINT = self.LASERPOINTS[PB][0]

            PB = PB - 1
            NEXTPOINT = self.LASERPOINTS[PB][0]
            if self.dist_point2point(POINT, NEXTPOINT) > self.GMAX:
                break
        PB = PB + 1

        LR = self.dist_point2point(self.LASERPOINTS[PB][0], self.LASERPOINTS[PF][0])
        PR = len(self.LASERPOINTS[PB:PF])

        if(LR >= self.LMIN) and (PR > self.PMIN):
            self.LINE_PARAMS = line_eq
            m, b = self.lineForm_G2SI(line_eq[0], line_eq[1], line_eq[2])
            self.two_points = self.line_2pts(m, b)
            self.LINE_SEGMENT.append((self.LASERPOINTS[PB + 1][0], self.LASERPOINTS[PF - 1][0]))
            return [self.LASERPOINTS[PB:PF], self.two_points, (self.LASERPOINTS[PB + 1][0], self.LASERPOINTS[PF - 1][0]), PF, line_eq, (m, b)]
        else:
            return False








