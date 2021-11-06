from SLAM import env,sensors
import pygame
import math

environment = env.envBuilder((600, 1200))
environment.originalMap = environment.map.copy()
laser = sensors.LidarSensor(200, environment.originalMap, (0.5, 0.01))
environment.map.fill((0, 0, 0))
environment.infomap = environment.map.copy()
running = True
while running:
    sensorON = False
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
        environment.store_data(sensor_data)
        environment.display_data()

    # blit the imfomat, which shows the obstacles sensor sensed
    environment.map.blit(environment.infomap, (0, 0))
    pygame.display.update()
