from abc import ABC, abstractmethod
import json
import socket
from .robot import Robot


class IConnection(ABC):
    """Abstract interface for remote robot communication."""

    @abstractmethod
    def send_and_receive(self, msg):
        """Sends a command and returns the sensor data."""
        ...


class SocketConnection(IConnection):
    def __init__(self, host, port):
        self._host = host
        self._port = port
        self._socket = None

    def __enter__(self):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.connect((self._host, self._port))
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self._socket:
            self._socket.close()

    def send_and_receive(self, msg):
        request = json.dumps(msg)
        self._socket.sendall(request.encode())
        response = self._socket.recv(1024)
        return json.loads(response.decode())


class SimulatedRobot(Robot):
    def __init__(self, connection, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._connection = connection

    def send_(self, msg):
        sensor_data = self._connection.send_and_receive(msg)
        self.update_(sensor_data)
