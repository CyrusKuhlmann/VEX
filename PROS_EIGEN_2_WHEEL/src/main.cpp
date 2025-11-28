#include "main.h"

#include "Eigen/Dense"
#include "api.h"
#include "odom.h"
Odom odom;
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
  pros::delay(500);
  pros::Task odom_task(std::bind(&Odom::odom_task_fn, &odom));
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  pros::MotorGroup left_motors({-16, -5, -10});
  pros::MotorGroup right_motors({11, 8, 20});

  while (true) {
    left_motors.move(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) /
                     127.0 * 100);
    right_motors.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) /
                      127.0 * 100);
    pros::delay(20);
  }
}
