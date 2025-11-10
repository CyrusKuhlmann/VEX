from .connection import SocketConnection, SimulatedRobot
from .actions import forward, turn_by, forwardPID, turn_byPID


def auton(robot):
    turn_byPID(robot, 360, speed=30)
    forwardPID(robot, 24, speed=100)
    turn_byPID(robot, -180, speed=30)
    forwardPID(robot, 24, speed=100)
    turn_byPID(robot, 180, speed=30)


def main():
    with SocketConnection("localhost", 65432) as conn:
        robot = SimulatedRobot(conn, 16.0, 10.0)  # width, length in inches
        auton(robot)
