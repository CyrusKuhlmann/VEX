#include "Eigen/Dense"
#include "api.h"
#include "odom.h"

class Actor {
 private:
  Odom& odom;
  pros::MotorGroup& left_motors;
  pros::MotorGroup& right_motors;

 public:
  Actor(Odom& odom_ref, pros::MotorGroup& left_motors_ref,
        pros::MotorGroup& right_motors_ref)
      : odom(odom_ref),
        left_motors(left_motors_ref),
        right_motors(right_motors_ref) {}
  void turn_by_degrees(double target_angle_degrees, double max_speed_percent);
  void turn_to_degrees(double target_angle_degrees, double max_speed_percent);
  void drive_straight(double target_distance_inches, double max_speed_percent);
  void go_to_point(Eigen::Matrix<double, 2, 1> target_xy,
                   double max_speed_percent);
};
