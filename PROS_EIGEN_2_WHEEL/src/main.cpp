#include "main.h"

#include "Eigen/Dense"
#include "actor.h"
#include "api.h"
#include "odom.h"
Odom odom;
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_motors({-16, -5, -10});
pros::MotorGroup right_motors({11, 8, 20});
Actor actor(odom, left_motors, right_motors);

void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  } else {
    pros::lcd::clear_line(2);
  }
}

void initialize() {
  pros::lcd::initialize();

  lateral_rot.reset_position();
  lateral_rot.set_data_rate(5);
  forward_rot.reset_position();
  forward_rot.set_data_rate(5);
  imu.reset(true);
  pros::delay(750);
  pros::Task odom_task(std::bind(&Odom::odom_task_fn, &odom));
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  // const Eigen::Matrix<double, 2, 1> bl_corner(0.0, 0.0);
  // const Eigen::Matrix<double, 2, 1> br_corner(48.0, 0.0);
  // const Eigen::Matrix<double, 2, 1> tr_corner(48.0, 48.0);
  // const Eigen::Matrix<double, 2, 1> tl_corner(0.0, 48.0);
  // actor.go_to_point(tl_corner, 50.0);
  // actor.go_to_point(br_corner, 50.0);
  // actor.go_to_point(tr_corner, 50.0);
  // actor.go_to_point(bl_corner, 50.0);
  // actor.turn_to_degrees(0.0, 50.0);
  actor.turn_by_degrees(90.0, 50.0);
  pros::delay(1000);
  actor.turn_by_degrees(-90.0, 50.0);
  pros::delay(1000);
  actor.drive_straight(12.0, 50.0);
}

void opcontrol() {
  autonomous();
  while (true) {
    left_motors.move(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) /
                     127.0 * 100);
    right_motors.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) /
                      127.0 * 100);
    pros::delay(20);
  }
}
