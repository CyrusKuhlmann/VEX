from enum import Enum

DRIVE_WHEEL_DIAMETER = 4  # in inches


class Direction(Enum):
    FORWARD = 1
    REVERSE = -1

    @classmethod
    def from_string(cls, s):
        return cls[s.upper()]


class Accessory:
    def __init__(self, port: int):
        self._port = port
        self.messages = []

    def send_(self, cmd: str, *args):
        msg = {
            "port": self._port,
            "cmd": cmd,
            "args": args,
        }
        self.messages.append(msg)

    def flush_(self):
        msgs = self.messages
        self.messages = []
        return msgs


class Motor(Accessory):
    def __init__(self, port, ratio, reversed):
        super().__init__(port)
        self._ratio = ratio
        self._reversed = reversed
        self._max_rpm = 450
        self._position = 0.0  # in degrees
        self._velocity = 0.0  # percentage of max RPM
        self._direction = Direction.FORWARD
        self._spinning = False

    def update_(self, sensor_data):
        if "position" in sensor_data:
            self._position = sensor_data["position"]
        if "velocity" in sensor_data:
            self._velocity = sensor_data["velocity"]
        if "direction" in sensor_data:
            self._direction = Direction.from_string(sensor_data["direction"])
        if "spinning" in sensor_data:
            self._spinning = sensor_data["spinning"]

    def position(self):
        """Return the current position of the motor in degrees."""
        return self._position

    def set_velocity(self, velocity):
        """Set the motor velocity as a percentage of max RPM."""
        self.send_("set_velocity", velocity)

    def spin(self, direction: Direction):
        """Spin the motor in the given direction."""
        self.send_("spin_motor", direction.name.lower())

    def stop(self):
        """Stop the motor."""
        self.send_("stop_motor")


class Inertial:
    def __init__(self, port):
        self._port = port
        self._rotation = 0.0  # in degrees

    def update_(self, sensor_data):
        self._rotation = sensor_data["rotation"]

    def rotation(self):
        """Return the current rotation of the inertial sensor in degrees."""
        return self._rotation


class Robot:
    def __init__(self, width, length):
        self._messages = []
        self._width = width
        self._length = length
        self._x = 0.0  # in inches
        self._y = 0.0  # in inches
        self._theta = 0.0  # in degrees
        self._clock = 0.0  # in seconds
        self._cheat = {"x": 0.0, "y": 0.0, "theta": 0.0}

        self.left_motor = Motor(port=1, ratio=1, reversed=False)
        self.right_motor = Motor(port=2, ratio=1, reversed=False)
        self.inertial = Inertial(port=3)

    def sleep(self, duration_ms: int):
        self._messages.append(
            {
                "port": -1,
                "cmd": "sleep",
                "args": [duration_ms],
            }
        )

    def update_(self, sensor_data):
        self._clock = sensor_data["clock"]
        self.left_motor.update_(sensor_data["left_motor"])
        self.right_motor.update_(sensor_data["right_motor"])
        self.inertial.update_(sensor_data["inertial"])
        self._cheat = sensor_data["cheat"]

    def flush_(self):
        msgs = self._messages
        self._messages = []
        msgs.extend(self.left_motor.flush_())
        msgs.extend(self.right_motor.flush_())
        return msgs
