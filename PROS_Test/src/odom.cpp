#include "odom.hpp"

#include <cmath>

#include "api.h"

pros::Rotation left_rot(1);
pros::Rotation right_rot(2);
pros::Rotation back_rot(3);

pros::IMU imu_sensor(4);

const double DRIVE_WHEEL_DIAMETER = 3.25;
const double DRIVE_WHEEL_CIRCUMFERENCE = DRIVE_WHEEL_DIAMETER * M_PI;
const double TRACKING_WHEEL_DIAMETER = 2.0;
const double TRACKING_WHEEL_CIRCUMFERENCE = TRACKING_WHEEL_DIAMETER * M_PI;
const double TL = 5.75;
const double TR = 5.75;
const double TB = 5.0;

Robot robot;

static double degrees_to_inches(double degrees, double wheel_circumference) {
  return (degrees / 360.0) * wheel_circumference;
}

static double constrain_angle(double angle) {
  while (angle > 2 * M_PI) {
    angle -= 2 * M_PI;
  }
  while (angle < 0) {
    angle += 2 * M_PI;
  }
  return angle;
}

void Robot::init() {
  left_rot.reset_position();
  right_rot.reset_position();
  back_rot.reset_position();

  imu_sensor.reset(true);

  prev_L = 0.0;
  prev_R = 0.0;
  prev_B = 0.0;
}

void Robot::update(double dt) {
  L = degrees_to_inches(left_rot.get_position(), TRACKING_WHEEL_CIRCUMFERENCE);
  R = degrees_to_inches(right_rot.get_position(), TRACKING_WHEEL_CIRCUMFERENCE);
  B = degrees_to_inches(back_rot.get_position(), TRACKING_WHEEL_CIRCUMFERENCE);

  double dL = L - prev_L;
  double dR = R - prev_R;
  double dB = B - prev_B;

  prev_L = L;
  prev_R = R;
  prev_B = B;

  double dTheta_odom = (dR - dL) / (TL + TR);
  double dY_local = (dL + dR) / 2.0;
  double dX_local = dB - (dTheta_odom * TB);

  double gyro_rate = imu_sensor.get_gyro_rate().z * (M_PI / 180.0);

  double theta_pred = theta + (gyro_rate * dt);
  KF_P += KF_Q;
  double KF_K = KF_P / (KF_P + KF_R);
  theta = theta_pred + KF_K * (dTheta_odom - (theta_pred - theta));
  theta = constrain_angle(theta);
  KF_P = (1 - KF_K) * KF_P;

  double avg_theta = theta + (dTheta_odom / 2.0);
  double cos_theta = cos(avg_theta);
  double sin_theta = sin(avg_theta);
  double dX_global = (dX_local * cos_theta) - (dY_local * sin_theta);
  double dY_global = (dX_local * sin_theta) + (dY_local * cos_theta);

  x += dX_global;
  y += dY_global;
}

void odom_task(void* param) {
  const int dt_ms = 10;
  const double dt = dt_ms / 1000.0;
  while (true) {
    robot.update(dt);
    pros::delay(dt_ms);
  }
}

void start_odometry_task() { pros::Task t(odom_task, nullptr, "odom_task"); }