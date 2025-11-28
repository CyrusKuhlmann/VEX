#pragma once

#include <cmath>

#include "Eigen/Dense"
#include "api.h"
#include "pod.h"

// Global rotation and IMU objects
extern pros::Rotation lateral_rot;
extern pros::Rotation forward_rot;
extern pros::Imu imu;

const double TRACKING_WHEEL_DIAMETER_INCHES = 2.0;
const double FORWARD_CORRECTION_FACTOR = 0.914346;  // inches per radian
const double LATERAL_CORRECTION_FACTOR = 7.883524;  // inches per radian
const double IMU_CORRECTION_FACTOR = 1.0059;  // multiplier to correct IMU drift

class Odom {
 private:
  OdomPod forward_pod;
  OdomPod lateral_pod;
  // Robot orientation
  double delta_theta_degrees;
  double theta_degrees;
  double prev_theta_degrees;
  double average_theta_degrees;

  // Robot position
  Eigen::Matrix<double, 2, 1> xy;
  Eigen::Matrix<double, 2, 1> prev_xy;
  double rad_to_deg(double rad);
  double deg_to_rad(double deg);
  void update_raw_values();
  void update_xy();
  void debug(int i);

 public:
  // Constructor
  Odom()
      : forward_pod(TRACKING_WHEEL_DIAMETER_INCHES, FORWARD_CORRECTION_FACTOR),
        lateral_pod(TRACKING_WHEEL_DIAMETER_INCHES, LATERAL_CORRECTION_FACTOR),
        delta_theta_degrees(0.0),
        theta_degrees(0.0),
        prev_theta_degrees(0.0),
        average_theta_degrees(0.0),
        xy(Eigen::Matrix<double, 2, 1>::Zero()),
        prev_xy(Eigen::Matrix<double, 2, 1>::Zero()) {}

  // Methods
  void odom_task_fn();
  double get_x_inches();
  double get_y_inches();
  double get_theta_degrees();
};
