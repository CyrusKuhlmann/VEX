from vexpy.sim.robot import Robot
from vexpy.sim.viz import PyGameVisualizer
from vexpy.sim.server import Server
from vexpy.sim.messages import MessageHandler

import logging

logging.basicConfig(level=logging.INFO)


def main():
    visualizer = PyGameVisualizer()
    robot = Robot(16.0, 10.0)  # width, length in inches
    handler = MessageHandler(robot)
    server = Server()

    def handle(msg):
        visualizer.draw(robot)
        return handler.process_message(msg)

    server.start(handle)
