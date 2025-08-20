// ----------------------------------------------------------------------------
// 
//  Module:       main.cpp
//  Author:       cyrus
//  Created:      6/28/2025, 12:26:42 PM
//  Description:  V5 project
//
// ----------------------------------------------------------------------------

#include "vex.h"
#include <cmath>

using namespace vex;

brain Brain;
motor leftMotorA(PORT1, ratio18_1, false);
motor leftMotorB(PORT11, ratio18_1, false);
motor_group left_motors(leftMotorA, leftMotorB);

motor rightMotorA(PORT10, ratio18_1, true);
motor rightMotorB(PORT20, ratio18_1, true);
motor_group right_motors(rightMotorA, rightMotorB);

motor motor1(PORT16, ratio18_1, false);

inertial inertial1(PORT17);

controller controller_1(primary);

void straight(double target_inches, double max_speed = 50) {
  double time_elapsed = 0.0;
  // Reset motor encoders
  left_motors.setPosition(0, degrees);
  right_motors.setPosition(0, degrees);

  // --- Constants ---
  const double wheel_circumference = 12.566; // 4" wheel
  const double gear_ratio = 20.0 / 48.0;     // external gearing
  const double target_degrees = (target_inches * gear_ratio / wheel_circumference) * 360.0;

  // --- PID Gains ---
  double Kp_dist = 0.325; // forward PID proportional
  double Ki_dist = 0.009; // small integral term for bias
  double Kd_dist = 0.038;  // derivative for overshoot control

  double Kp_drift = 0.300; // drift correction proportional
  double Kd_drift = 0.042; // drift derivative

  // --- PID State ---
  double dist_error_prev = 0;
  double dist_integral = 0;

  double drift_error_prev = 0;

  // --- Control ---
  double slew_rate = 2.0; // % per loop
  double output = 0;

  double dt = 0.02;
  int stable_count = 0;
  const int required_stable_cycles = 5;
  const double tolerance = 10; // degrees

  while (stable_count < required_stable_cycles) {
    // --- Positions ---
    double left_pos = left_motors.position(degrees);
    double right_pos = right_motors.position(degrees);
    double avg_pos = (left_pos + right_pos) / 2.0;

    // --- Distance PID ---
    double dist_error = target_degrees - avg_pos;
    dist_integral += dist_error * dt;
    double dist_derivative = (dist_error - dist_error_prev) / dt;
    dist_error_prev = dist_error;

    double dist_output = (Kp_dist * dist_error) + (Ki_dist * dist_integral) + (Kd_dist * dist_derivative);

    // --- Slew Rate ---
    if (dist_output > output + slew_rate) output += slew_rate;
    else if (dist_output < output - slew_rate) output -= slew_rate;
    else output = dist_output;

    // Clamp output
    if (output > max_speed) output = max_speed;
    if (output < -max_speed) output = -max_speed;

    // --- Drift Correction ---
    double drift_error = left_pos - right_pos;
    double drift_derivative = (drift_error - drift_error_prev) / dt;
    drift_error_prev = drift_error;

    double drift_correction = ((Kp_drift * drift_error) + (Kd_drift * drift_derivative));

    // --- Motor Speeds ---
    double left_speed = output - drift_correction;
    double right_speed = output + drift_correction;

    left_motors.spin((left_speed >= 0) ? forward : reverse, fabs(left_speed), percent);
    right_motors.spin((right_speed >= 0) ? forward : reverse, fabs(right_speed), percent);

    // --- Exit Condition ---
    if (fabs(dist_error) < tolerance) stable_count++;
    else stable_count = 0;

    time_elapsed += dt;

    wait(dt, seconds);
  }

  // Stop motors
  left_motors.stop(brake);
  right_motors.stop(brake);
}

void turn(double target_degrees, double max_speed = 38.5) {
  // Reset encoders
  left_motors.setPosition(0, degrees);
  right_motors.setPosition(0, degrees);

  // --- PID Gains ---
  double Kp_angle = 0.250;
  double Ki_angle = 0.007;
  double Kd_angle = 0.029;

  double Kp_drift = 0.210;
  double Kd_drift = 0.035;

  // --- PID State ---
  double angle_error_prev = 0;
  double angle_integral = 0;

  double drift_error_prev = 0;

  // --- Control ---
  double output = 0;
  double slew_rate = 2.0; // % per loop
  double dt = 0.02;

  int stable_count = 0;
  const int required_stable_cycles = 5;
  const double tolerance = 3; // degrees

  while (stable_count < required_stable_cycles) {
    // --- Measured turn angle ---
    double left_pos = left_motors.position(degrees);
    double right_pos = right_motors.position(degrees);
    // Turning angle estimate: difference between sides

    double current_angle = ((left_pos - right_pos) / 2) * 21/36;

    // --- PID for angle ---
    double angle_error = target_degrees - current_angle;
    angle_integral += angle_error * dt;
    double angle_derivative = (angle_error - angle_error_prev) / dt;
    angle_error_prev = angle_error;

    double angle_output = ((Kp_angle * angle_error) + (Ki_angle * angle_integral) + (Kd_angle * angle_derivative));

    // --- Slew rate ---
    if (angle_output > output + slew_rate) output += slew_rate;
    else if (angle_output < output - slew_rate) output -= slew_rate;
    else output = angle_output;

    // Clamp
    if (output > max_speed) output = max_speed;
    if (output < -max_speed) output = -max_speed;

    // --- Drift correction ---
    double drift_error = fabs(left_pos) - fabs(right_pos);
    double drift_derivative = (drift_error - drift_error_prev) / dt;
    drift_error_prev = drift_error;

    double drift_correction = (Kp_drift * drift_error) + (Kd_drift * drift_derivative);

    // --- Motor Speeds ---
    double left_speed = output - drift_correction;
    double right_speed = -output - drift_correction; // negative for opposite spin

    left_motors.spin((left_speed >= 0) ? forward : reverse, fabs(left_speed), percent);
    right_motors.spin((right_speed >= 0) ? forward : reverse, fabs(right_speed), percent);

    // --- Exit Condition ---
    if (fabs(angle_error) < tolerance) stable_count++;
    else stable_count = 0;

    wait(dt, seconds);
  }

  // Stop motors with brake hold
  left_motors.stop(brake);
  right_motors.stop(brake);
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

void spin_motor1(int speed1) {
  motor1.setVelocity(speed1, percent);
  motor1.spin(forward);
}

void inertial_turn(double target_angle, double max_speed = 50) {
  inertial1.setHeading(0, degrees); // Reset heading

  double inertial_turn_kp = 0.2; 

  while (fabs(inertial1.heading(degrees) - target_angle) > 4.0) {
    double current_angle = inertial1.heading(degrees);
    double error = target_angle - current_angle;

    double output = error * inertial_turn_kp;
    if (output > max_speed) output = max_speed;
    if (output < -max_speed) output = -max_speed;

    left_motors.spin((output >= 0) ? forward : reverse, fabs(output), percent);
    right_motors.spin((output >= 0) ? reverse : forward, fabs(output), percent);

    wait(20, msec);
  }

  left_motors.stop(brake);
  right_motors.stop(brake);
}

int main() {
  inertial_turn(-90); // Spin motor1 at 100% speed
}

