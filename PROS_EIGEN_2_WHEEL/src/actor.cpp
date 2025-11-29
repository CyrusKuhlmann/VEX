#include "actor.h"

void Actor::turn_by_degrees(double delta_degrees, double max_speed_percent) {
  pros::lcd::print(2, "Turning by %.2f degrees", delta_degrees);

  // ----- PID GAIN CONSTANTS -----
  const double KP = 0.6267;
  const double KD = 0.0925;
  const double KI = 0.01;

  // Feed-forward (base torque) — tweak between 5–12%
  const double KF = 8.5;  // percent power

  const double TOLERANCE_DEGREES = 1.0;

  const int DT_MS = 67;
  const double DT = DT_MS / 1000.0;

  // ----- SLEW RATE -----
  // Limit how fast the command can increase.
  const double SLEW_UP = 9.0;     // % per cycle (accelerating)
  const double SLEW_DOWN = 11.0;  // % per cycle (decelerating)
  double last_output_percent = 0;

  // starting info
  double stable_cycles = 0;
  double start_rotation = odom.get_theta_degrees();
  double target = start_rotation + delta_degrees;

  // PID variables
  double error = 0, prev_error = 0;
  double integral = 0, derivative = 0;

  while (stable_cycles < 5) {
    double current = odom.get_theta_degrees();
    error = target - current;

    // Integral windup protection
    if (std::abs(error) < 15) {
      integral += error * DT;
      integral = std::clamp(integral, -200.0, 200.0);
    } else {
      integral = 0;
    }

    // Derivative (filtered)
    double raw_deriv = (error - prev_error) / DT;
    derivative = 0.7 * derivative + 0.3 * raw_deriv;
    prev_error = error;

    // PID + FEEDFORWARD
    double output_percent = KP * error + KI * integral + KD * derivative;

    // Add feed-forward torque
    if (error > 0)
      output_percent += KF;
    else if (error < 0)
      output_percent -= KF;

    // Clamp to max
    output_percent =
        std::clamp(output_percent, -max_speed_percent, max_speed_percent);

    // --------- SLEW LIMITING ---------
    double delta = output_percent - last_output_percent;

    if (delta > SLEW_UP)
      output_percent = last_output_percent + SLEW_UP;
    else if (delta < -SLEW_DOWN)
      output_percent = last_output_percent - SLEW_DOWN;

    last_output_percent = output_percent;

    // Convert % → motor units
    double output = output_percent / 100.0 * 127;

    // Anti-deadzone
    if (std::abs(output) < 10 && std::abs(error) > TOLERANCE_DEGREES) {
      output = (output > 0 ? 10 : -10);
    }

    // Send to motors
    left_motors.move(output);
    right_motors.move(-output);

    // Check completion stability
    if (std::abs(error) < TOLERANCE_DEGREES) {
      stable_cycles++;
    } else {
      stable_cycles = 0;
    }

    pros::delay(DT_MS);
  }

  // Stop and hold
  left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  left_motors.move(0);
  right_motors.move(0);
}

void Actor::drive_straight(double target_distance_inches,
                           double max_speed_percent) {
  const double KP_DIST = 2.0;
  const double KI_DIST = 0.95;
  const double KD_DIST = 0.1;

  const double KP_HEADING = 0.85;

  const double K_F = 0.10;  // Feed-forward: adds torque, esp. near the end

  const double TOLERANCE_INCHES = 1.0;
  const double TOLERANCE_HEADING_DEG = 1.0;

  const double DT = 0.067;
  const double MAX_SLEW = 3.0;  // % change allowed per loop

  int stable_cycles = 0;

  // ----- Starting pose -----
  Eigen::Vector2d start_xy = odom.get_xy_inches();
  double start_theta_deg = odom.get_theta_degrees();

  double error_dist = target_distance_inches, prev_error_dist = error_dist;
  double integral_dist = 0, derivative_dist = 0;
  double derivative_filtered = 0;

  double last_left = 0, last_right = 0;

  while (stable_cycles < 5) {
    Eigen::Vector2d current_xy = odom.get_xy_inches();
    double current_theta_deg = odom.get_theta_degrees();

    // Distance PID
    double traveled_distance = (current_xy - start_xy).norm();
    error_dist = target_distance_inches - traveled_distance;

    // Integral windup protection
    if (std::abs(error_dist) < 20) {
      integral_dist += error_dist * DT;
      integral_dist = std::clamp(integral_dist, -100.0, 100.0);
    } else {
      integral_dist = 0;
    }

    // Derivative with smoothing
    double raw_derivative = (error_dist - prev_error_dist) / DT;
    derivative_filtered = 0.8 * derivative_filtered + 0.2 * raw_derivative;
    derivative_dist = derivative_filtered;
    prev_error_dist = error_dist;

    // Feed-forward (helps torque near the end)
    double feed_forward = K_F * (error_dist > 0 ? 1 : -1);

    double output_percent = KP_DIST * error_dist + KI_DIST * integral_dist +
                            KD_DIST * derivative_dist + feed_forward;

    // Heading correction
    double heading_error_deg = start_theta_deg - current_theta_deg;
    double heading_correction = KP_HEADING * heading_error_deg;

    // Motor outputs BEFORE slew
    double left_output_raw = output_percent + heading_correction;
    double right_output_raw = output_percent - heading_correction;

    // Slew rate
    auto apply_slew = [&](double target, double last) {
      double delta = target - last;
      if (std::abs(delta) > MAX_SLEW)
        return last + (delta > 0 ? MAX_SLEW : -MAX_SLEW);
      return target;
    };

    double left_output = apply_slew(left_output_raw, last_left);
    double right_output = apply_slew(right_output_raw, last_right);

    last_left = left_output;
    last_right = right_output;

    // Clamp to max
    left_output =
        std::clamp(left_output, -max_speed_percent, max_speed_percent);
    right_output =
        std::clamp(right_output, -max_speed_percent, max_speed_percent);

    // Convert % → motor units
    left_motors.move(left_output / 100.0 * 127);
    right_motors.move(right_output / 100.0 * 127);

    // Check completion stability
    if (std::abs(error_dist) < TOLERANCE_INCHES &&
        std::abs(heading_error_deg) < TOLERANCE_HEADING_DEG) {
      stable_cycles++;
    } else {
      stable_cycles = 0;
    }

    pros::delay(DT * 1000);
  }

  left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  left_motors.move(0);
  right_motors.move(0);
}

void Actor::go_to_point(Eigen::Matrix<double, 2, 1> target_xy,
                        double max_speed_percent) {
  double tries = 0;

  while (tries < 1) {
    double target_inches = odom.distance_to_point_inches(target_xy);

    if (target_inches < 0.5) {
      break;
    }

    double angle_difference_degrees = odom.angle_to_point_degrees(target_xy);

    turn_by_degrees(angle_difference_degrees, max_speed_percent);
    drive_straight(target_inches, max_speed_percent);
    tries++;
  }
}

void Actor::turn_to_degrees(double target_angle_degrees,
                            double max_speed_percent) {
  pros::lcd::print(1, "Turning to %.2f degrees", target_angle_degrees);
  turn_by_degrees(odom.angle_to_heading_degrees(target_angle_degrees),
                  max_speed_percent);
}
