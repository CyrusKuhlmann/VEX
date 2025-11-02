import json
import socket


def send(s, msg):
    s.sendall(msg.encode())
    data = s.recv(1024)
    return json.loads(data.decode())


def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect(("localhost", 65432))
        send(s, "set_velocity|left|50")
        intertial = 0
        while intertial < 360:
            sensors = send(s, "spin_motor|left|forward")
            print(sensors)
            intertial = sensors["inertial"]["rotation"]
            send(s, "sleep|20")
        send(s, "stop_motor|left")
