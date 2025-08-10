# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       cyrus                                                        #
# 	Created:      8/9/2025, 2:41:55 PM                                         #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

from vex import *
import time

brain = Brain()
controller = Controller(PRIMARY)

left_motors = MotorGroup(Motor(Ports.PORT1, GearSetting.RATIO_18_1, False), Motor(Ports.PORT11, GearSetting.RATIO_18_1, False))
right_motors = MotorGroup(Motor(Ports.PORT10, GearSetting.RATIO_18_1, True), Motor(Ports.PORT20, GearSetting.RATIO_18_1, True))

WHEEL_DIAMETER_INCHES = 4
WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_INCHES * math.pi
GEAR_RATIO = 7/3  


def inches_to_degrees(inches):
    return (inches / WHEEL_CIRCUMFERENCE) * 360 * GEAR_RATIO


def drive_forward(speed, distance):
    Kp = 0.6
    Ki = 0.0005
    Kd = 0.25
    slew_limit = 5           
    integral_limit = 50      
    deadband = 0.5           
    

    target_deg = inches_to_degrees(distance)
    left_motors.set_position(0, DEGREES)
    right_motors.set_position(0, DEGREES)
    
    error, integral, derivative = 0, 0, 0
    last_error = 0
    output = 0
    last_output = 0
    
    while True:
        left_pos = left_motors.position(DEGREES)
        right_pos = right_motors.position(DEGREES)
        avg_pos = (left_pos + right_pos) / 2
        
        error = target_deg - avg_pos
        if abs(error) < deadband:
            break
        
        # Integral windup protection
        if abs(error) < integral_limit:
            integral += error
        else:
            integral = 0
        
        derivative = error - last_error
        output = (Kp * error) + (Ki * integral) + (Kd * derivative)
        
        # Slew rate limit
        if output - last_output > slew_limit:
            output = last_output + slew_limit
        elif last_output - output > slew_limit:
            output = last_output - slew_limit
        
        # Clamp to speed limit
        output = max(min(output, speed), -speed)
        
        left_motors.spin(FORWARD, output, PERCENT)
        right_motors.spin(FORWARD, output, PERCENT)
        
        last_error = error
        last_output = output
        time.sleep(0.02)
    
    left_motors.stop(BRAKE)
    right_motors.stop(BRAKE)


def drive_backward(distance, speed=50):
    drive_forward(distance, -speed) 


def turn_in_place(degrees, speed=50):
    Kp = 0.8
    Ki = 0.0006
    Kd = 0.3
    slew_limit = 6
    integral_limit = 40
    deadband = 0.5
    
    track_width_inches = 15.75
    turn_circumference = track_width_inches * math.pi
    wheel_degrees = inches_to_degrees((turn_circumference * degrees) / 360)
    
    left_motors.set_position(0, DEGREES)
    right_motors.set_position(0, DEGREES)
    
    error = 0
    integral = 0
    derivative = 0

    last_error = 0
    output = 0
    last_output = 0
    
    while True:
        left_pos = left_motors.position(DEGREES)
        right_pos = -right_motors.position(DEGREES)  # Reverse sign for turning
        avg_pos = (left_pos + right_pos) / 2
        
        error = wheel_degrees - avg_pos
        if abs(error) < deadband:
            break
        
        if abs(error) < integral_limit:
            integral += error
        else:
            integral = 0
        
        derivative = error - last_error
        output = (Kp * error) + (Ki * integral) + (Kd * derivative)
        
        if output - last_output > slew_limit:
            output = last_output + slew_limit
        elif last_output - output > slew_limit:
            output = last_output - slew_limit
        
        output = max(min(output, speed), -speed)
        
        left_motors.spin(FORWARD, output, PERCENT)
        right_motors.spin(REVERSE, output, PERCENT)
        
        last_error = error
        last_output = output
        time.sleep(0.02)
    
    left_motors.stop(BRAKE)
    right_motors.stop(BRAKE)


drive_forward(50, 10)     # Drive forward 24 inches
turn_in_place(40, 180)     # Turn right 90 degrees
drive_backward(50, 10)    # Drive backward 12 inches
