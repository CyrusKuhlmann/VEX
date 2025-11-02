import json
import time
import logging
from .motor import Direction

logger = logging.getLogger(__name__)


class MessageHandler:
    def __init__(self, robot):
        self.robot = robot

    def _get_motor_by_port(self, port):
        if port == 1:
            return self.robot.left_motor
        elif port == 2:
            return self.robot.right_motor
        else:
            raise ValueError(f"Unknown motor port: {port}")

    def process_message(self, msg):
        logger.info(f"Processing message: {msg}")
        try:
            data = json.loads(msg)
            cmd = data["cmd"]
            args = data["args"]

            if cmd == "set_velocity":
                velocity = float(args[0])
                motor = self._get_motor_by_port(data["port"])
                motor.velocity = velocity

            elif cmd == "stop_motor":
                motor = self._get_motor_by_port(data["port"])
                motor.stop()

            elif cmd == "spin_motor":
                direction = args[0]
                dir = Direction.from_string(direction)
                motor = self._get_motor_by_port(data["port"])
                motor.spin(dir)

            elif cmd == "sleep":
                duration_ms = int(args[0])
                while duration_ms > 20:
                    self.robot.step(0.02)
                    time.sleep(0.02)
                    duration_ms -= 20
                self.robot.step(duration_ms / 1000.0)
                time.sleep(duration_ms / 1000.0)
        except Exception as e:
            logger.error(f"Error processing message '{msg}': {e}")
            raise e

        return json.dumps(self.robot.sense())
