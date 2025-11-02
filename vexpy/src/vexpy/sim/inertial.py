class InertialSensor:
    def __init__(self, id):
        self._id = id
        self._rotation = 0.0  # in degrees

    @property
    def rotation(self):
        return self._rotation

    @rotation.setter
    def rotation(self, value):
        self._rotation = value

    def sense(self):
        return {"rotation": self.rotation}
