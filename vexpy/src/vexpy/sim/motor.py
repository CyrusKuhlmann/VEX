from enum import Enum
import random
import math

MAX_RPM = 450
MAX_ACCELERATION = 8000000000000000.0


class Direction(Enum):
    FORWARD = 1
    REVERSE = -1

    @classmethod
    def from_string(cls, s):
        return cls[s.upper()]


class Motor:
    def __init__(self, id, max_rpm=MAX_RPM):
        self._id = id
        self._max_rpm = max_rpm
        self._position = 0.0  # in degrees
        self._velocity_pct = 0.0  # percentage of max RPM
        self._target_velocity_pct = 0.0
        self._target_speed_pct = 0.0
        self._direction = Direction.FORWARD
        self._spinning = False

    @property
    def velocity(self):  # getVelocity
        return abs(self._velocity_pct)

    @velocity.setter
    def velocity(self, value):  # setVelocity
        if value < 0 or value > 100:
            raise ValueError("Velocity must be between 0 and 100")
        if self._spinning:
            self._target_velocity_pct = value * self._direction.value
        self._target_speed_pct = value

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, value):
        self._position = value

    def spin(self, direction: Direction):
        self._direction = direction
        self._spinning = True
        self._target_velocity_pct = self._target_speed_pct * direction.value

    def stop(self):
        self._spinning = False
        self._target_velocity_pct = 0.0

    def step(self, dt):
        acceleration = (self._target_velocity_pct - self._velocity_pct) / dt
        if abs(acceleration) > MAX_ACCELERATION:
            acceleration = math.copysign(MAX_ACCELERATION, acceleration)
        self._velocity_pct += acceleration * dt
        delta_degrees = 0.0
        if self._spinning:
            rpm = (self._velocity_pct / 100.0) * self._max_rpm
            degrees_per_second = (rpm * 360) / 60
            delta_degrees = degrees_per_second * dt
            self._position += delta_degrees
        return delta_degrees

    def sense(self):
        return {
            "position": self.position,
        }
