import math

from .motor import Motor
from .inertial import InertialSensor

DRIVE_WHEEL_DIAMETER = 4  # in inches


class Robot:
    """
    A simple differential drive robot model with two motors and an inertial sensor.
    """

    def __init__(self, width, length):
        """
        Initialize the robot with given width and length.

        :param width: Width of the robot in inches.
        :param length: Length of the robot in inches.
        """
        self._width = width
        self._length = length

        self._x = 0.0  # in inches
        self._y = 0.0  # in inches
        self._theta = 0.0  # in degrees
        self._clock = 0.0  # in seconds

        self.left_motor = Motor("left")
        self.right_motor = Motor("right")
        self.inertial = InertialSensor("inertial")

    @property
    def width(self):
        return self._width

    @property
    def length(self):
        return self._length

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @property
    def theta(self):
        return self._theta

    @property
    def clock(self):
        return self._clock

    def step(self, dt):
        dl = self.left_motor.step(dt) * DRIVE_WHEEL_DIAMETER * math.pi / 360
        dr = self.right_motor.step(dt) * DRIVE_WHEEL_DIAMETER * math.pi / 360

        d_theta = (dl - dr) / self.width  # in radians
        ds = (dl + dr) / 2
        theta_rad = math.radians(self._theta)
        theta_avg = theta_rad + (d_theta / 2)

        dx = ds * math.sin(theta_avg)
        dy = ds * math.cos(theta_avg)

        self._x += dx
        self._y += dy
        self._theta += math.degrees(d_theta)
        self.inertial.rotation = self._theta

        self._clock += dt

    def sense(self):
        return {
            "clock": self.clock,
            "left_motor": self.left_motor.sense(),
            "right_motor": self.right_motor.sense(),
            "inertial": self.inertial.sense(),
            "cheat": {
                "x": self.x,
                "y": self.y,
                "theta": self.theta,
            },
        }
