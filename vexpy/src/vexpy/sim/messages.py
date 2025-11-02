import json
import time
import logging
from .motor import Direction

logger = logging.getLogger(__name__)


class MessageHandler:
    def __init__(self, robot):
        self.robot = robot

    def process_message(self, msg):
        logger.info(f"Processing message: {msg}")
        try:
            cmd = msg.split("|")[0]
            params = msg.split("|")[1:]

            if cmd == "set_velocity":
                motor_id = params[0]
                velocity = float(params[1])
                if motor_id == "left":
                    self.robot.left_motor.velocity = velocity
                elif motor_id == "right":
                    self.robot.right_motor.velocity = velocity
                else:
                    raise ValueError(f"Unknown motor ID: {motor_id}")

            elif cmd == "stop_motor":
                motor_id = params[0]
                if motor_id == "left":
                    self.robot.left_motor.stop()
                elif motor_id == "right":
                    self.robot.right_motor.stop()
                else:
                    raise ValueError(f"Unknown motor ID: {motor_id}")

            elif cmd == "spin_motor":
                motor_id = params[0]
                direction = params[1]
                dir = Direction.from_string(direction)
                if motor_id == "left":
                    self.robot.left_motor.spin(dir)
                elif motor_id == "right":
                    self.robot.right_motor.spin(dir)
                else:
                    raise ValueError(f"Unknown motor ID: {motor_id}")

            elif cmd == "sleep":
                duration_ms = int(params[0])
                self.robot.step(duration_ms / 1000.0)
                time.sleep(duration_ms / 1000.0)

        except Exception as e:
            logger.error(f"Error processing message '{msg}': {e}")

        return json.dumps(self.robot.sense())
