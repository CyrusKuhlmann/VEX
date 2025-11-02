import json
import socket
from .robot import Robot
from .robot import Direction


def send(s, messages):
    msg = json.dumps(messages)
    s.sendall(msg.encode())
    data = s.recv(1024)
    return json.loads(data.decode())


def main():
    robot = Robot(16.0, 10.0)  # width, length in inches
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect(("localhost", 65432))
        robot.left_motor.set_velocity(50)
        robot.left_motor.spin(Direction.FORWARD)
        send(s, robot.flush_())
        intertial = 0
        while intertial < 360:
            robot.sleep(20)
            sensors = send(s, robot.flush_())
            print(sensors)
            intertial = sensors["inertial"]["rotation"]
        robot.left_motor.stop()
        send(s, robot.flush_())
