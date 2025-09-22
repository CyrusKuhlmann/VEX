# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       cyrus                                                        #
# 	Created:      6/28/2025, 12:26:42 PM                                       #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

brain = Brain()
left_motors = MotorGroup(Motor(Ports.PORT1), Motor(Ports.PORT11))
right_motors = MotorGroup(Motor(Ports.PORT10), Motor(Ports.PORT20))
controller_1 = Controller(PRIMARY)


def straight(target_inches, speed=50):
    left_motors.set_position(0, RotationUnits.DEG)
    right_motors.set_position(0, RotationUnits.DEG)
    target_inches_corrected = target_inches * (20/48)
    target_degrees = (target_inches_corrected / 12.566) * 360

    Kp = .08
    Kd = 0
    x = 0

    ramp_speed = 0
    ramp_step = .5

    dt = 0.0075
    prev_error = 0

    tolerance = 10
    stable_count = 0
    required_stable_cycles = 20

    while stable_count < required_stable_cycles:
        if ramp_speed < speed:
            ramp_speed += ramp_step
            ramp_speed = min(ramp_speed, speed)
        
        if ((left_motors.position() + right_motors.position()) / 2) > (abs(target_degrees) - 60):
            if ramp_speed >= 10:
                ramp_speed -= ramp_step

        error = left_motors.position() - right_motors.position()
        derivative = (error - prev_error) / dt
        prev_error = error
        avg_pos = (left_motors.position() - right_motors.position()) / 2

        brain.screen.print(error, "|")

        correction = Kp * error + Kd * derivative
        left_speed = ramp_speed - correction
        right_speed = ramp_speed + correction

        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))

        left_direction = DirectionType.FORWARD if left_speed >= 0 else DirectionType.REVERSE
        right_direction = DirectionType.FORWARD if right_speed >= 0 else DirectionType.REVERSE

        left_motors.spin(left_direction, abs(left_speed), VelocityUnits.PERCENT)
        right_motors.spin(right_direction, abs(right_speed), VelocityUnits.PERCENT)

        left_pos_close = abs(left_motors.position() - target_degrees) < tolerance
        right_pos_close = abs(right_motors.position() - target_degrees) < tolerance

        if left_pos_close and right_pos_close:
            stable_count += 1
        else:
            stable_count = 0 

        # --- Draw error on screen as a vertical line ---
        y_center = 120
        scale = 2.75
        y = int(y_center - (avg_pos) * scale)
        y = max(0, min(239, y))


        # Draw X-axis (error = 0 line)
        if x == 0:
            for xi in range(480):  # full screen width
                brain.screen.draw_pixel(xi, y_center)

        target_error = target_degrees  # If you want to show where the target is on the error plot
        y_target = int(y_center - (target_error) * scale)
        y_target = max(0, min(239, y_target))
        for xi in range(480):
            brain.screen.draw_pixel(xi, y_target)

        # Draw Y-axis (time = 0 line)
        brain.screen.draw_pixel(0, y)  # every frame, plot current point on y-axis for context

        # Draw error point
        if x < 480:
            brain.screen.draw_pixel(x, y)
            x += 1

        wait(dt, TimeUnits.SECONDS)
    
    right_motors.stop()
    left_motors.stop()

def turn(said_target_angle, turn_speed=25):
    left_motors.set_position(0, RotationUnits.DEG)
    right_motors.set_position(0, RotationUnits.DEG)

    target_angle = said_target_angle * (180 / 110)
    K = .09

    ramp_speed = 0
    ramp_step = 1

    direction = 1 if target_angle > 0 else -1

    while abs(left_motors.position()) < abs(target_angle) or abs(right_motors.position()) < abs(target_angle):
        if ramp_speed < turn_speed:
            ramp_speed += ramp_step
            ramp_speed = min(ramp_speed, turn_speed)

        error = abs(left_motors.position()) - abs(right_motors.position())
        brain.screen.print(error,"|")
        left_speed = ramp_speed - K * error
        right_speed = ramp_speed + K * error

        left_speed = max(0, min(100, left_speed))
        right_speed = max(0, min(100, right_speed))

        if abs(left_motors.position()) < abs(target_angle):
            left_motors.spin(
            DirectionType.FORWARD if direction > 0 else DirectionType.REVERSE,
            ramp_speed,
            VelocityUnits.PERCENT,
        )
        else:
            left_motors.stop()

        if abs(right_motors.position()) < abs(target_angle):
            right_motors.spin(
            DirectionType.REVERSE if direction > 0 else DirectionType.FORWARD,
            ramp_speed,
            VelocityUnits.PERCENT,
        )
        else:
            right_motors.stop() 

        wait(.02, TimeUnits.SECONDS)

    right_motors.stop()
    left_motors.stop()

def user_control():
    left_stick_y = controller_1.axis3.position()
    right_stick_y = controller_1.axis2.position()

    left_motors.set_velocity(-left_stick_y, VelocityUnits.PERCENT)
    right_motors.set_velocity(right_stick_y, VelocityUnits.PERCENT)

    left_motors.spin(DirectionType.FORWARD)
    right_motors.spin(DirectionType.FORWARD)

    wait(0.02, TimeUnits.SECONDS)

if __name__ == "__main__":
    while True:
        user_control()