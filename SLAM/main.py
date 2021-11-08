from SLAM import env, sensors, feature
import pygame
import random
import math


def random_color():
    levels = range(32, 256, 32)
    return tuple(random.choice(levels) for _ in range(3))


FeatureMap = feature.featureDetection()
environment = env.envBuilder((600, 1200))
originalMap = environment.map.copy()
laser = sensors.LidarSensor(200, originalMap, (0.5, 0.01))
environment.map.fill((255, 255, 255))
environment.infomap = environment.map.copy()
originalMap = environment.map.copy()
running = True
FEATURE_DETECTION = True
BREAK_POINT_INDEX = 0
while running:
    environment.infomap = originalMap.copy()
    FEATURE_DETECTION = True
    BREAK_POINT_INDEX = 0
    ENDPOINTS = [0, 0]
    sensorON = False
    PREDICTED_POINTS_TO_DRAW = []
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    if pygame.mouse.get_focused():
        sensorON = True
    elif not pygame.mouse.get_focused():
        sensorON = False
    if sensorON:
        position = pygame.mouse.get_pos()
        # sensor is at the position of the mouse
        laser.position = position
        # from the mouse, sense the obstacles and displays it
        sensor_data = laser.sense_walls()
        FeatureMap.laser_point_set(sensor_data)
        while BREAK_POINT_INDEX < (FeatureMap.NP - FeatureMap.PMIN):
            seedSeg = FeatureMap.seed_segment_detection(laser.position, BREAK_POINT_INDEX)
            if seedSeg == False:
                break
            else:
                seedSegment = seedSeg[0]
                PREDICTED_POINTS_TO_DRAW = seedSegment[1]
                INDICES = seedSeg[2]
                results = FeatureMap.seed_segment_grow(INDICES, BREAK_POINT_INDEX)
                if results == False:
                    BREAK_POINT_INDEX = INDICES[1]
                    continue
                else:
                    line_eq = results[1]
                    m, c = results[5]
                    line_seg = results[0]
                    OUTERMOST = results[2]
                    BREAK_POINT_INDEX = results[3]

                    ENDPOINTS[0] = FeatureMap.projection_pt2ln(OUTERMOST[0], m, c)
                    ENDPOINTS[1] = FeatureMap.projection_pt2ln(OUTERMOST[1], m, c)

                    COLOR = random_color()
                    for point in line_seg:
                        environment.infomap.set_at((int(point[0][0]), int(point[0][1])), (0, 255, 0))
                        pygame.draw.circle(environment.infomap, COLOR, (int(point[0][0]), int(point[0][1])), 2, 0)
                    pygame.draw.line(environment.infomap, (255, 0, 0), ENDPOINTS[0], ENDPOINTS[1], 2)
                    environment.store_data(sensor_data)

    # blit the imfomat, which shows the obstacles sensor sensed
    environment.map.blit(environment.infomap, (0, 0))
    pygame.display.update()
