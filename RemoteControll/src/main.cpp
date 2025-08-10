// ----------------------------------------------------------------------------
// 
//  Module:       main.cpp
//  Author:       cyrus
//  Created:      6/28/2025, 12:26:42 PM
//  Description:  V5 project
//
// ----------------------------------------------------------------------------

#include "vex.h"

using namespace vex;

brain Brain;
motor leftMotorA(PORT1, ratio18_1, false);
motor leftMotorB(PORT11, ratio18_1, false);
motor_group left_motors(leftMotorA, leftMotorB);

motor rightMotorA(PORT10, ratio18_1, true);
motor rightMotorB(PORT20, ratio18_1, true);
motor_group right_motors(rightMotorA, rightMotorB);

controller controller_1(primary);

void straight(double target_inches, double speed = 50) {
  left_motors.setPosition(0, degrees);
  right_motors.setPosition(0, degrees);

  double target_inches_corrected = target_inches * (20.0 / 48.0);
  double target_degrees = (target_inches_corrected / 12.566) * 360.0;

  double Kp = 0.08;
  double Kd = 0;
  int x = 0;

  double ramp_speed = 0;
  double ramp_step = 0.5;

  double dt = 0.0075;
  double prev_error = 0;

  double tolerance = 10;
  int stable_count = 0;
  int required_stable_cycles = 1;

  while (stable_count < required_stable_cycles) {
    if (ramp_speed < speed) {
      ramp_speed += ramp_step;
      if (ramp_speed > speed) ramp_speed = speed;
    }

    if (((left_motors.position(degrees) + right_motors.position(degrees)) / 2.0) >
        (fabs(target_degrees) - 60)) {
      if (ramp_speed >= 10) {
        ramp_speed -= ramp_step;
      }
    }

    double error = left_motors.position(degrees) - right_motors.position(degrees);
    double derivative = (error - prev_error) / dt;
    prev_error = error;
    double avg_pos = (left_motors.position(degrees) - right_motors.position(degrees)) / 2.0;

    Brain.Screen.print("%f |", error);

    double correction = Kp * error + Kd * derivative;
    double left_speed = ramp_speed - correction;
    double right_speed = ramp_speed + correction;

    if (left_speed > 100) left_speed = 100;
    if (left_speed < -100) left_speed = -100;
    if (right_speed > 100) right_speed = 100;
    if (right_speed < -100) right_speed = -100;

    directionType left_direction = (left_speed >= 0) ? forward : reverse;
    directionType right_direction = (right_speed >= 0) ? forward : reverse;

    left_motors.spin(left_direction, fabs(left_speed), percent);
    right_motors.spin(right_direction, fabs(right_speed), percent);

    bool left_pos_close = fabs(left_motors.position(degrees) - target_degrees) < tolerance;
    bool right_pos_close = fabs(right_motors.position(degrees) - target_degrees) < tolerance;

    if (left_pos_close && right_pos_close) {
      stable_count++;
    } else {
      stable_count = 0;
    }

    // --- Draw error on screen as a vertical line ---
    int y_center = 120;
    double scale = 2.75;
    int y = (int)(y_center - (avg_pos) * scale);
    if (y < 0) y = 0;
    if (y > 239) y = 239;

    // Draw X-axis (error = 0 line)
    if (x == 0) {
      for (int xi = 0; xi < 480; xi++) {
        Brain.Screen.drawPixel(xi, y_center);
      }
    }

    double target_error = target_degrees;
    int y_target = (int)(y_center - (target_error) * scale);
    if (y_target < 0) y_target = 0;
    if (y_target > 239) y_target = 239;
    for (int xi = 0; xi < 480; xi++) {
      Brain.Screen.drawPixel(xi, y_target);
    }

    // Draw Y-axis (time = 0 line)
    Brain.Screen.drawPixel(0, y);

    // Draw error point
    if (x < 480) {
      Brain.Screen.drawPixel(x, y);
      x++;
    }

    wait(dt, seconds);
  }

  right_motors.stop();
  left_motors.stop();
}

void turn(double said_target_angle, double turn_speed = 25) {
  left_motors.setPosition(0, degrees);
  right_motors.setPosition(0, degrees);

  double target_angle = said_target_angle * (180.0 / 110.0);
  double K = 0.09;

  double ramp_speed = 0;
  double ramp_step = 1;

  int direction = (target_angle > 0) ? 1 : -1;

  while (fabs(left_motors.position(degrees)) < fabs(target_angle) ||
         fabs(right_motors.position(degrees)) < fabs(target_angle)) {

    if (ramp_speed < turn_speed) {
      ramp_speed += ramp_step;
      if (ramp_speed > turn_speed) ramp_speed = turn_speed;
    }

    double error = fabs(left_motors.position(degrees)) - fabs(right_motors.position(degrees));
    Brain.Screen.print("%f |", error);

    double left_speed = ramp_speed - K * error;
    double right_speed = ramp_speed + K * error;

    if (left_speed < 0) left_speed = 0;
    if (left_speed > 100) left_speed = 100;
    if (right_speed < 0) right_speed = 0;
    if (right_speed > 100) right_speed = 100;

    if (fabs(left_motors.position(degrees)) < fabs(target_angle)) {
      left_motors.spin((direction > 0) ? forward : reverse, ramp_speed, percent);
    } else {
      left_motors.stop();
    }

    if (fabs(right_motors.position(degrees)) < fabs(target_angle)) {
      right_motors.spin((direction > 0) ? reverse : forward, ramp_speed, percent);
    } else {
      right_motors.stop();
    }

    wait(0.02, seconds);
  }

  right_motors.stop();
  left_motors.stop();
}

void user_control() {
  double left_stick_y = controller_1.Axis3.position();
  double right_stick_y = controller_1.Axis2.position();

  left_motors.setVelocity(left_stick_y, percent);
  right_motors.setVelocity(right_stick_y, percent);

  left_motors.spin(forward);
  right_motors.spin(forward);

  wait(0.02, seconds);
}

int main() {
  straight(40);
  turn(180);
  straight(40);
}
