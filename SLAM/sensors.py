import pygame
import math
import numpy as np


def add_noise(distance, theta, sigma):
    mean = np.array([distance, theta])
    covariance = np.diag(sigma ** 2)
    distance, theta = np.random.multivariate_normal(mean, covariance)
    distance = max(distance, 0)
    theta = max(theta, 0)
    return [distance, theta]


class LidarSensor:

    def __init__(self, range, map, noise):
        self.range = range
        self.frequency = 4
        self.sigma = np.array([noise[0], noise[1]])
        self.position = (0, 0)
        self.map = map
        self.Width, self.Height = pygame.display.get_surface().get_size()
        self.obstacles = []

    def distance_euclid(self, obstacle_pos):
        return math.sqrt((obstacle_pos[0] - self.position[0]) ** 2 +
                         (obstacle_pos[1] - self.position[1]) ** 2)

    def sense_walls(self):
        data = []
        x1, y1 = self.position[0], self.position[1]
        for theta in np.linspace(0, 2 * math.pi, 60, False):
            x2, y2 = (x1 + self.range * math.cos(theta), y1 - self.range * math.sin(theta))
            for i in range(0, 100):
                u = i / 100
                x = int(x2 * u + x1 * (1 - u))
                y = int(y2 * u + y1 * (1 - u))

                if 0 < x < self.Width and 0 < y < self.Height:
                    color = self.map.get_at((x, y))
                    if (color[0], color[1], color[2]) == (0, 0, 0):
                        distance = self.distance_euclid((x, y))
                        output = add_noise(distance, theta, self.sigma)
                        output.append(self.position)
                        # store the measurement
                        data.append(output)
                        # stop extend the laser beam because laser hit the wall already
                        break
        # sensor at this point finish the full half circle
        if len(data) > 0:
            return data
        else:
            # no wall is sensed around the robot
            return False
