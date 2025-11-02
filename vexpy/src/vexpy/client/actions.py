import math
from .robot import Direction, DRIVE_WHEEL_DIAMETER


def drive_wheel_degrees_to_inches(degrees):
    return degrees * (DRIVE_WHEEL_DIAMETER * math.pi / 360)


def turn_by(robot, target_rotation, speed=50):
    initial_rotation = robot.inertial.rotation()
    desired_rotation = initial_rotation + target_rotation

    if target_rotation > 0:
        robot.left_motor.set_velocity(speed)
        robot.right_motor.set_velocity(speed)
        robot.left_motor.spin(Direction.FORWARD)
        robot.right_motor.spin(Direction.REVERSE)

        while robot.inertial.rotation() < desired_rotation:
            robot.sleep(20)

    else:
        robot.left_motor.set_velocity(speed)
        robot.right_motor.set_velocity(speed)
        robot.left_motor.spin(Direction.REVERSE)
        robot.right_motor.spin(Direction.FORWARD)

        while robot.inertial.rotation() > desired_rotation:
            robot.sleep(20)

    robot.left_motor.stop()
    robot.right_motor.stop()


def forward(robot, target_distance, speed=50):
    initial_left = robot.left_motor.position()
    initial_right = robot.right_motor.position()

    robot.left_motor.set_velocity(speed)
    robot.right_motor.set_velocity(speed)
    robot.left_motor.spin(Direction.FORWARD)
    robot.right_motor.spin(Direction.FORWARD)

    traveled_distance = 0.0
    while traveled_distance < target_distance:
        robot.sleep(20)
        dl = robot.left_motor.position() - initial_left
        dr = robot.right_motor.position() - initial_right
        avg_distance = (dl + dr) / 2
        traveled_distance = drive_wheel_degrees_to_inches(avg_distance)

    robot.left_motor.stop()
    robot.right_motor.stop()
