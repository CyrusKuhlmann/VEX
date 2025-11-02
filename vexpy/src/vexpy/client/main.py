from .connection import SocketConnection, SimulatedRobot
from .actions import forward, turn_by


def auton(robot):
    turn_by(robot, 360, speed=30)
    forward(robot, 24, speed=50)
    turn_by(robot, -180, speed=30)
    forward(robot, 24, speed=20)
    turn_by(robot, 180, speed=10)


def main():
    with SocketConnection("localhost", 65432) as conn:
        robot = SimulatedRobot(conn, 16.0, 10.0)  # width, length in inches
        auton(robot)
