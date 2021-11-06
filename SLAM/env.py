import math
import pygame


class envBuilder:
    def __init__(self, MapDimensions):
        pygame.init()
        self.points = []
        self.Walls = pygame.image.load("Map2.png")
        self.mapHeight, self.mapWidth = MapDimensions
        self.MapWindowTitle = 'Radar Sensor Simulation Window'
        pygame.display.set_caption(self.MapWindowTitle)
        self.map = pygame.display.set_mode((self.mapWidth, self.mapHeight))
        self.map.blit(self.Walls, (0, 0))

        # Colors
        self.black = (0, 0, 0)
        self.grey = (70, 70, 70)
        self.blue = (0, 0, 255)
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)
        self.white = (255, 255, 255)

    def AD2pos(self, distance, theta, robotposition):
        x = distance * math.cos(theta) + robotposition[0]
        y = -distance * math.sin(theta) + robotposition[1]
        return (int(x), int(y))

    def store_data(self, data):
        print(len(self.points))
        if data != False:
            for element in data:
                point = self.AD2pos(element[0], element[1], element[2])
                if point not in self.points:
                    self.points.append(point)

    def display_data(self):
        self.infomap = self.map.copy()
        for point in self.points:
            self.infomap.set_at((int(point[0]), int(point[1])), self.red)
