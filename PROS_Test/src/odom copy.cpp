#include <cmath>

#include "api.h"
#include "odom.hpp"

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

class Robot {
 public:
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;

  double L = 0.0;
  double R = 0.0;
  double B = 0.0;

  double prev_L = 0.0;
  double prev_R = 0.0;
  double prev_B = 0.0;

  double IMU_theta = 0.0;

  static double degrees_to_inches(double degrees, double wheel_circumference) {
    return (degrees / 360.0) * wheel_circumference;
  };

  void update() {
    L = robot.degrees_to_inches(left_rot.get_position(),
                                TRACKING_WHEEL_CIRCUMFERENCE);
    R = robot.degrees_to_inches(right_rot.get_position(),
                                TRACKING_WHEEL_CIRCUMFERENCE);
    B = robot.degrees_to_inches(back_rot.get_position(),
                                TRACKING_WHEEL_CIRCUMFERENCE);

    double dL = L - prev_L;
    double dR = R - prev_R;
    double dB = B - prev_B;
    double dTheta = (dR - dL) / (TL + TR);
    theta += dTheta;

    double dX_local = dB - (dTheta * TB);
    double dY_local = (dL + dR) / 2.0;

    double avg_theta = theta - (dTheta / 2.0);
    double dX_global =
        (dX_local * cos(avg_theta)) - (dY_local * sin(avg_theta));
    double dY_global =
        (dX_local * sin(avg_theta)) + (dY_local * cos(avg_theta));

    x += dX_global;
    y += dY_global;

    prev_L = L;
    prev_R = R;
    prev_B = B;
  }
};
Robot robot;

void odom_task(void* param) {
  while (true) {
    robot.update();
    pros::delay(20);
  }
}
