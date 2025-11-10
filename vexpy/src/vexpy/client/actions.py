import math
from .robot import Direction, DRIVE_WHEEL_DIAMETER

INERTIAL_TURN_KP = 10
INERTIAL_TURN_KI = 0.05
INERTIAL_TURN_KD = 0.07
DT = 20  # milliseconds

KP_DIST = 0.5
KI_DIST = 0.0
KD_DIST = 0.0

KP_DRIFT = 0.0
KD_DRIFT = 0.0

SLEW_RATE = 2


def drive_wheel_degrees_to_inches(degrees):
    return degrees * (DRIVE_WHEEL_DIAMETER * math.pi / 360)


def turn_byPID(robot, target_rotation, speed=50):
    stable_cycles = 0
    initial_rotation = robot.inertial.rotation()
    desired_rotation = initial_rotation + target_rotation
    inertial_turn_integral = 0.0

    inertial_turn_prev_error = 0.0

    while stable_cycles < 5:
        current_angle = robot.inertial.rotation()

        error = desired_rotation - current_angle
        inertial_turn_derivative = (error - inertial_turn_prev_error) / (DT / 1000.0)
        inertial_turn_prev_error = error

        inertial_turn_integral += error * (DT / 1000.0)

        output = (
            (error * INERTIAL_TURN_KP)
            + (inertial_turn_integral * INERTIAL_TURN_KI)
            + (inertial_turn_derivative * INERTIAL_TURN_KD)
        )

        if output > speed:
            output = speed
        elif output < -speed:
            output = -speed

        robot.left_motor.set_velocity(abs(output))
        robot.right_motor.set_velocity(abs(output))

        robot.left_motor.spin(Direction.FORWARD if output > 0 else Direction.REVERSE)
        robot.right_motor.spin(Direction.REVERSE if output > 0 else Direction.FORWARD)

        if abs(error) < 0.5:
            stable_cycles += 1
        else:
            stable_cycles = 0

        robot.sleep(DT)

    robot.left_motor.stop()
    robot.right_motor.stop()


def forwardPID(robot, target_distance, speed=100):
    kp_dist = KP_DIST
    ki_dist = KI_DIST
    kd_dist = KD_DIST
    initial_left_pos = drive_wheel_degrees_to_inches(robot.left_motor.position())
    initial_right_pos = drive_wheel_degrees_to_inches(robot.right_motor.position())

    dist_error_prev = 0.0
    dist_integral = 0.0

    drift_error_prev = 0.0

    stable_count = 0

    startAngle = robot.inertial.rotation()
    output = 0.0
    while stable_count < 10:
        left_pos = drive_wheel_degrees_to_inches(robot.left_motor.position())
        right_pos = drive_wheel_degrees_to_inches(robot.right_motor.position())

        delta_left = left_pos - initial_left_pos
        delta_right = right_pos - initial_right_pos
        avg_delta = (delta_left + delta_right) / 2.0

        dist_error = target_distance - avg_delta
        dist_integral += dist_error * (DT / 1000.0)
        dist_derivative = (dist_error - dist_error_prev) / (DT / 1000.0)
        dist_error_prev = dist_error

        dist_output = (
            (kp_dist * dist_error)
            + (ki_dist * dist_integral)
            + (kd_dist * dist_derivative)
        )

        if dist_output > output + SLEW_RATE:
            output += SLEW_RATE
        elif dist_output < output - SLEW_RATE:
            output -= SLEW_RATE
        else:
            output = dist_output

        if output > speed:
            output = speed
        elif output < -speed:
            output = -speed

        drift_error = robot.inertial.rotation() - startAngle
        drift_derivative = (drift_error - drift_error_prev) / (DT / 1000.0)
        drift_error_prev = drift_error

        drift_correction = (KP_DRIFT * drift_error) + (KD_DRIFT * drift_derivative)

        left_speed = output - drift_correction
        right_speed = output + drift_correction

        robot.left_motor.set_velocity(abs(left_speed))
        robot.right_motor.set_velocity(abs(right_speed))

        robot.left_motor.spin(
            Direction.FORWARD if left_speed > 0 else Direction.REVERSE
        )
        robot.right_motor.spin(
            Direction.FORWARD if right_speed > 0 else Direction.REVERSE
        )

        if abs(dist_error) < 0.5:
            stable_count += 1
        else:
            stable_count = 0

        robot.sleep(DT)

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


def turn_by(robot, target_rotation, speed=50):
    initial_rotation = robot.inertial.rotation()
    desired_rotation = initial_rotation + target_rotation

    error = desired_rotation - robot.inertial.rotation()
    while abs(error) > 1.0:
        robot.left_motor.set_velocity(speed)
        robot.right_motor.set_velocity(speed)
        robot.left_motor.spin(Direction.FORWARD if error > 0 else Direction.REVERSE)
        robot.right_motor.spin(Direction.REVERSE if error > 0 else Direction.FORWARD)
        robot.sleep(20)
        error = desired_rotation - robot.inertial.rotation()

    robot.left_motor.stop()
    robot.right_motor.stop()
