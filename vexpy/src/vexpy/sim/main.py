from vexpy.sim.robot import Robot
from vexpy.sim.motor import Direction
from vexpy.sim.viz import PyGameVisualizer

import time
import pygame
import sys

DELTA_SECONDS = 0.02


def main():
    visualizer = PyGameVisualizer()
    robot = Robot(16.0, 10.0)  # width, length in inches
    robot.left_motor.velocity = 5.0  # percent
    robot.right_motor.velocity = 5.0  # percent
    robot.left_motor.spin(Direction.FORWARD)
    robot.right_motor.spin(Direction.FORWARD)
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit(0)
        visualizer.draw(robot)
        robot.step(DELTA_SECONDS)
        time.sleep(DELTA_SECONDS)  # Keep the window open
