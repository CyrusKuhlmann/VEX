from vexpy.sim.robot import Robot
from vexpy.sim.viz import PyGameVisualizer

import time
import pygame
import sys


def main():
    robot = Robot(16.0, 10.0)  # width, length in inches
    visualizer = PyGameVisualizer()

    visualizer.draw(robot)
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit(0)
        time.sleep(1)  # Keep the window open
        visualizer.draw(robot)
