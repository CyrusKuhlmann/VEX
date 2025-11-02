import json
import socket
from .robot import Robot, Direction


class SocketRobot(Robot):
    def __init__(self, socket, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._socket = socket

    def send_(self, messages):
        msg = json.dumps(messages)
        self._socket.sendall(msg.encode())
        data = self._socket.recv(1024)
        sensor_data = json.loads(data.decode())
        self.update_(sensor_data)


def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.connect(("localhost", 65432))
        robot = SocketRobot(sock, 16.0, 10.0)  # width, length in inches
        robot.left_motor.set_velocity(50)
        robot.left_motor.spin(Direction.FORWARD)
        while robot.inertial.rotation() < 360:
            robot.sleep(20)
        robot.left_motor.stop()
