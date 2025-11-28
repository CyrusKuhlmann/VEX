#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <Eigen/Dense>

#include "api.h"

// -------------------------
// Robot Class Declaration
// -------------------------
class Robot {
 public:
  // Constants
  const double TRACKING_WHEEL_DIAMETER = 2.0;
  const double TRACKING_WHEEL_CIRCUMFERENCE = TRACKING_WHEEL_DIAMETER * M_PI;
  const double TL = 5.75;
  const double TR = 5.75;
  const double TB = 5.0;

  // Global deltas
  double dl_global = 0.0;
  double dr_global = 0.0;
  double db_global = 0.0;

  // Pose variables
  double x = 0.0;
  double y = 0.0;
  double theta_odom = 0.0;         // degrees
  double theta_imu = 0.0;          // degrees
  double theta_fusion = 0.0;       // degrees
  double theta_fusion_prev = 0.0;  // degrees
  double dtheta_fusion = 0.0;      // degrees

  // IMU gyro
  double gyro_z = 0.0;

  // Tracking wheel readings
  double l = 0.0;
  double r = 0.0;
  double b = 0.0;
  double prev_l = 0.0;
  double prev_r = 0.0;
  double prev_b = 0.0;

  // Functions
  double wrapAngle(double angle);
  double deg_to_rad(double degrees);
  double rad_to_deg(double radians);
  void update_odom_theta();
  void update_position();
  void update_IMU();
};

// Global robot instance
extern Robot robot;

// -------------------------
// Kalman Filter Declaration
// -------------------------
class Kalman_Filter {
 private:
  static const int n = 1;
  static const int m = 1;
  static const int p = 2;

  const double dt = 0.02;

  // State & matrices
  Eigen::Matrix<double, n, 1> x_k_k_1_pred;
  Eigen::Matrix<double, n, 1> x_k_k_pred;
  Eigen::Matrix<double, m, 1> u_k;
  Eigen::Matrix<double, n, n> A;
  Eigen::Matrix<double, n, m> B;

  // Measurement
  Eigen::Matrix<double, p, 1> z_k;
  Eigen::Matrix<double, p, n> H;

  // Noise
  Eigen::Matrix<double, n, n> Q;
  Eigen::Matrix<double, p, p> R;

  // Covariance
  Eigen::Matrix<double, n, n> P_k_k_1;
  Eigen::Matrix<double, n, n> P_k_k;

  // Innovation
  Eigen::Matrix<double, p, 1> y_k;
  Eigen::Matrix<double, p, p> S_k;
  Eigen::Matrix<double, n, p> K_k;
  Eigen::Matrix<double, n, n> I;

 public:
  Eigen::Matrix<double, n, 1> x_k;

  void init();
  void update_measurement();
  void predict();
  void update();
  void advance();
};

// Global KF instance
extern Kalman_Filter kf;

// -------------------------
// Tasks
// -------------------------
void kalman_task();

#endif
