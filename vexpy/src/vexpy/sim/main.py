from vexpy.sim.robot import Robot
from vexpy.sim.motor import Direction
from vexpy.sim.viz import PyGameVisualizer
from vexpy.sim.server import Server
from vexpy.sim.messages import MessageHandler
import time
import pygame
import sys

DELTA_SECONDS = 0.02


def old():
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


def main():
    visualizer = PyGameVisualizer()
    robot = Robot(16.0, 10.0)  # width, length in inches
    handler = MessageHandler(robot)
    server = Server()

    def handle(msg):
        visualizer.draw(robot)
        return handler.process_message(msg)

    server.start(handle)
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit(0)
        visualizer.draw(robot)
        robot.step(DELTA_SECONDS)
        time.sleep(DELTA_SECONDS)  # Keep the window open
