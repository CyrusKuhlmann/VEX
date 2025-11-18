#include "main.h"

#include "api.h"
#include "odom.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  } else {
    pros::lcd::clear_line(2);
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  robot.init();
  start_odometry_task();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */

void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */

void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void autonomous() {
  for (int i = 0; i < 500; i++) {
    pros::lcd::print(0, "X: %.2f", robot.x);
    pros::lcd::print(1, "Y: %.2f", robot.y);
    pros::lcd::print(2, "Theta: %.2f", robot.theta);
    pros::delay(20);
  }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  pros::MotorGroup left_mg({5, -6, 7});  // Creates a motor group with forwards
                                         // ports 5 & 7 and reversed port 6
  pros::MotorGroup right_mg(
      {-8, 9, -10});  // Creates a motor group with forwards port 9 and reversed
                      // ports 8 & 10

  while (true) {
    pros::lcd::print(0, "X: %.2f", robot.x);
    pros::lcd::print(1, "Y: %.2f", robot.y);
    pros::lcd::print(2, "Theta: %.2f", robot.theta);

    // Arcade control scheme
    int dir = master.get_analog(
        ANALOG_LEFT_Y);  // Gets amount forward/backward from left joystick
    int turn = master.get_analog(
        ANALOG_RIGHT_X);        // Gets the turn left/right from right joystick
    left_mg.move(dir - turn);   // Sets left motor voltage
    right_mg.move(dir + turn);  // Sets right motor voltage
    pros::delay(20);            // Run for 20 ms then update
  }
}