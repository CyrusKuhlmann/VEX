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
            commands = json.loads(msg)
            for data in commands:
                cmd = data["cmd"]
                args = data["args"]
                motor_id = "left" if data["port"] == 1 else "right"

                if cmd == "set_velocity":
                    velocity = float(args[0])
                    if motor_id == "left":
                        self.robot.left_motor.velocity = velocity
                    elif motor_id == "right":
                        self.robot.right_motor.velocity = velocity
                    else:
                        raise ValueError(f"Unknown motor ID: {motor_id}")

                elif cmd == "stop_motor":
                    if motor_id == "left":
                        self.robot.left_motor.stop()
                    elif motor_id == "right":
                        self.robot.right_motor.stop()
                    else:
                        raise ValueError(f"Unknown motor ID: {motor_id}")

                elif cmd == "spin_motor":
                    direction = args[0]
                    dir = Direction.from_string(direction)
                    if motor_id == "left":
                        self.robot.left_motor.spin(dir)
                    elif motor_id == "right":
                        self.robot.right_motor.spin(dir)
                    else:
                        raise ValueError(f"Unknown motor ID: {motor_id}")

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
            # return json.dumps({"error": str(e)})
            raise e

        return json.dumps(self.robot.sense())
