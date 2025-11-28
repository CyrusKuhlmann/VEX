#include "odom.h"

#include <cmath>

#include "Eigen/Dense"
#include "api.h"

pros::v5::Rotation lateral_rot(3);
pros::v5::Rotation forward_rot(6);
pros::v5::Imu imu(1);

double Odom::rad_to_deg(double rad) { return rad * (180.0 / M_PI); }
double Odom::deg_to_rad(double deg) { return deg * (M_PI / 180.0); }

void Odom::update_raw_values() {
  prev_theta_degrees = theta_degrees;
  theta_degrees = imu.get_rotation() * IMU_CORRECTION_FACTOR;
  average_theta_degrees = (theta_degrees + prev_theta_degrees) / 2.0;
  delta_theta_degrees = theta_degrees - prev_theta_degrees;
  int forward_centidegrees = forward_rot.get_position();
  int lateral_centidegrees = lateral_rot.get_position();
  forward_pod.update(forward_centidegrees, deg_to_rad(delta_theta_degrees));
  lateral_pod.update(lateral_centidegrees, deg_to_rad(delta_theta_degrees));
}

void Odom::update_xy() {
  prev_xy = xy;
  Eigen::Matrix<double, 2, 1> delta_xy;
  delta_xy << lateral_pod.get_corrected_delta_inches(),
      forward_pod.get_corrected_delta_inches();
  Eigen::Matrix<double, 2, 2> R_average_theta;
  double average_theta_radians = deg_to_rad(average_theta_degrees);
  R_average_theta << cos(average_theta_radians), -sin(average_theta_radians),
      sin(average_theta_radians), cos(average_theta_radians);
  xy = prev_xy + R_average_theta * delta_xy;
}
void Odom::debug(int i) {
  pros::lcd::print(0, "X: %.2f in", xy(0, 0));
  pros::lcd::print(1, "Y: %.2f in", xy(1, 0));
  pros::lcd::print(2, "Theta: %.2f deg", theta_degrees);
  // print out the raw values for rotation sensors
  // pros::lcd::print(5, "L: %.2f in", s_l_corrected);
  pros::lcd::print(3, "Uncorrected L: %.2f in", lateral_pod.get_raw_inches());
  pros::lcd::print(4, "Corrected L: %.2f in",
                   lateral_pod.get_corrected_inches());
  pros::lcd::print(5, "Uncorrected F: %.2f in", forward_pod.get_raw_inches());
  pros::lcd::print(6, "Corrected F: %.2f in",
                   forward_pod.get_corrected_inches());
}

void Odom::odom_task_fn() {
  int i = 0;
  while (true) {
    update_raw_values();
    update_xy();
    debug(i);
    pros::delay(50);
    i++;
  }
}
