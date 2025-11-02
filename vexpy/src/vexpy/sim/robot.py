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

    def step(self, dt):
        dl = self.left_motor.step(dt) * DRIVE_WHEEL_DIAMETER * math.pi / 360
        dr = self.right_motor.step(dt) * DRIVE_WHEEL_DIAMETER * math.pi / 360

        d_theta = (dr - dl) / self.width  # in radians
        ds = (dl + dr) / 2
        theta_avg = self.theta + (d_theta / 2)

        dx = ds * math.cos(theta_avg)
        dy = ds * math.sin(theta_avg)

        self._x += dx
        self._y += dy
        self._theta += math.degrees(d_theta)
        self.inertial.rotation = self._theta
