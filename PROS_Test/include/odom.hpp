#pragma once
#include "pros/apix.h"

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

  double KF_Q = 0.001;
  double KF_R = 0.01;
  double KF_P = 0.01;

  void init();
  void update(double dt);
};

extern Robot robot;

void start_odometry_task();