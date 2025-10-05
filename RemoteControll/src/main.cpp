// ----------------------------------------------------------------------------
// 
//  Module:       main.cpp
//  Author:       cyrus
//  Created:      6/28/2025, 12:26:42 PM
//  Description:  V5 project
//
// ----------------------------------------------------------------------------

#include "vex.h"
#include <cmath>
#include <iostream>
#include <deque>

using namespace vex;

brain Brain;
motor leftMotorA(PORT1, ratio18_1, true);
motor leftMotorB(PORT11, ratio18_1, true);
motor_group left_motors(leftMotorA, leftMotorB);

motor rightMotorA(PORT10, ratio18_1, false);
motor rightMotorB(PORT20, ratio18_1, false);
motor_group right_motors(rightMotorA, rightMotorB);

motor motor1(PORT16, ratio18_1, false);

motor backTop(PORT13, ratio18_1, false);
motor backMiddle(PORT2, ratio18_1, false);
motor backBottom(PORT12, ratio18_1, false);

motor frontTop(PORT3, ratio18_1, true);
motor frontMiddle(PORT19, ratio18_1, true);
motor frontBottom(PORT15, ratio18_1, true);

motor_group frontAndBackMotors(frontTop, backTop, backBottom);

inertial inertial1(PORT17);

optical colorSensor(PORT14);

controller controller_1(primary);

enum Color { RED, BLUE, NONE };

Color myTeam = NONE;

class Ball {
	private:
		Color color;
		int observationTime;
	public:
		Ball(Color c, int t) : color(c), observationTime(t) {}
		Color getColor() const { return color; }
		int getObservationTime() const { return observationTime; }
};

std::ostream& operator<<(std::ostream& os, const Ball& b) {
	os << (b.getColor() == RED ? "RED" : "BLUE") << " at " << b.getObservationTime();
	return os;
}

std::deque<Ball> intakeq;
std::deque<Ball> ejectq;



bool isNearObject = false;
bool isPrevNearObject = false;

Color observedColor = NONE;
Color prevObservedColor = NONE;


#define MODE_OFF 0
#define MODE_SCORE_BOTTOM 1
#define MODE_SCORE_MIDDLE 2
#define MODE_SCORE_TOP 3
#define MODE_INTAKE 4
#define MODE_SHOOT_OUT_BALLS 5

int towerMode = MODE_OFF;

int mainTime = 0;


void straight(double target_inches, double max_speed = 50) {
  double time_elapsed = 0.0;
  // Reset motor encoders
  left_motors.setPosition(0, degrees);
  right_motors.setPosition(0, degrees);

  // --- Constants ---
  const double wheel_circumference = 12.96; // 4" wheel
  const double gear_ratio = 36.0 / 84.0;     // external gearing
  const double target_degrees = (target_inches * gear_ratio / wheel_circumference) * 360.0;

  // --- PID Gains ---
  double Kp_dist = 0.2; // forward PID proportional
  double Ki_dist = 0.001; // small integral term for bias
  double Kd_dist = 0.01;  // derivative for overshoot control

  double Kp_drift = 0.200; // drift correction proportional
  double Kd_drift = 0.021; // drift derivative

  // --- PID State ---
  double dist_error_prev = 0;
  double dist_integral = 0;

  double drift_error_prev = 0;

  // --- Control ---
  double slew_rate = 2.0; // % per loop
  double output = 0;

  double dt = 0.02;
  int stable_count = 0;
  const int required_stable_cycles = 10;
  const double tolerance = 7.5; // degrees

  double startAngle = inertial1.rotation(degrees);
  while (stable_count < required_stable_cycles) {
    // --- Positions ---
    double left_pos = left_motors.position(degrees);
    double right_pos = right_motors.position(degrees);
    double avg_pos = (left_pos + right_pos) / 2.0;

    // --- Distance PID ---
    double dist_error = target_degrees - avg_pos;
    dist_integral += dist_error * dt;
    double dist_derivative = (dist_error - dist_error_prev) / dt;
    dist_error_prev = dist_error;

    double dist_output = (Kp_dist * dist_error) + (Ki_dist * dist_integral) + (Kd_dist * dist_derivative);

    // --- Slew Rate ---
    if (dist_output > output + slew_rate) output += slew_rate;
    else if (dist_output < output - slew_rate) output -= slew_rate;
    else output = dist_output;

    // Clamp output
    if (output > max_speed) output = max_speed;
    if (output < -max_speed) output = -max_speed;

    // --- Drift Correction ---
    double drift_error = inertial1.rotation(degrees) - startAngle;
    double drift_derivative = (drift_error - drift_error_prev) / dt;
    drift_error_prev = drift_error;

    double drift_correction = ((Kp_drift * drift_error) + (Kd_drift * drift_derivative));

    // --- Motor Speeds ---
    double left_speed = (output - drift_correction);
    double right_speed = (output + drift_correction);

    left_motors.spin((left_speed >= 0) ? forward : reverse, fabs(left_speed), percent);
    right_motors.spin((right_speed >= 0) ? forward : reverse, fabs(right_speed), percent);

    // --- Exit Condition ---
    if (fabs(dist_error) < tolerance) stable_count++;
    else stable_count = 0;

    time_elapsed += dt;

    vex::wait(dt, seconds);

  }

  // Stop motors
  left_motors.stop(brake);
  right_motors.stop(brake);
}

void turn(double target_degrees, double max_speed = 38.5) {
  // Reset encoders
  left_motors.setPosition(0, degrees);
  right_motors.setPosition(0, degrees);

  // --- PID Gains ---
  double Kp_angle = 0.250;
  double Ki_angle = 0.007;
  double Kd_angle = 0.029;

  double Kp_drift = 0.210;
  double Kd_drift = 0.035;

  // --- PID State ---
  double angle_error_prev = 0;
  double angle_integral = 0;

  double drift_error_prev = 0;

  // --- Control ---
  double output = 0;
  double slew_rate = 2.0; // % per loop
  double dt = 0.02;

  int stable_count = 0;
  const int required_stable_cycles = 5;
  const double tolerance = 3; // degrees

  while (stable_count < required_stable_cycles) {
    // --- Measured turn angle ---
    double left_pos = left_motors.position(degrees);
    double right_pos = right_motors.position(degrees);
    // Turning angle estimate: difference between sides

    double current_angle = ((left_pos - right_pos) / 2) * 21/36;

    // --- PID for angle ---
    double angle_error = target_degrees - current_angle;
    angle_integral += angle_error * dt;
    double angle_derivative = (angle_error - angle_error_prev) / dt;
    angle_error_prev = angle_error;

    double angle_output = ((Kp_angle * angle_error) + (Ki_angle * angle_integral) + (Kd_angle * angle_derivative));

    // --- Slew rate ---
    if (angle_output > output + slew_rate) output += slew_rate;
    else if (angle_output < output - slew_rate) output -= slew_rate;
    else output = angle_output;

    // Clamp
    if (output > max_speed) output = max_speed;
    if (output < -max_speed) output = -max_speed;

    // --- Drift correction ---
    double drift_error = fabs(left_pos) - fabs(right_pos);
    double drift_derivative = (drift_error - drift_error_prev) / dt;
    drift_error_prev = drift_error;

    double drift_correction = (Kp_drift * drift_error) + (Kd_drift * drift_derivative);

    // --- Motor Speeds ---
    double left_speed = output - drift_correction;
    double right_speed = -output - drift_correction; // negative for opposite spin

    left_motors.spin((left_speed >= 0) ? forward : reverse, fabs(left_speed), percent);
    right_motors.spin((right_speed >= 0) ? forward : reverse, fabs(right_speed), percent);

    // --- Exit Condition ---
    if (fabs(angle_error) < tolerance) stable_count++;
    else stable_count = 0;

    vex::wait(dt, seconds);
  }

  // Stop motors with brake hold
  left_motors.stop(brake);
  right_motors.stop(brake);
}

void user_control(double speed = 50) {
  double left_stick_y = controller_1.Axis3.position();
  if (left_stick_y > speed) {
    left_stick_y = speed;
  }
  double right_stick_y = controller_1.Axis2.position();
  if (right_stick_y > speed) {
    right_stick_y = speed;
  }

  left_motors.setVelocity(left_stick_y, velocityUnits(percent));
  right_motors.setVelocity(right_stick_y, velocityUnits(percent));

  

  left_motors.spin(forward);
  right_motors.spin(forward);

}

void spin_motor1(int speed1) {
  motor1.setVelocity(speed1, percent);
  motor1.spin(forward);
}

void inertial_turn(double target_angle, double max_speed = 50) {

  double inertial_turn_kp = 0.235; 

  double final = target_angle + inertial1.rotation(degrees);

    // --- Graph settings ---
  Brain.Screen.clearScreen();
  Brain.Screen.setPenColor(white);
  Brain.Screen.setFillColor(black);
  Brain.Screen.setFont(mono20);
  Brain.Screen.printAt(10, 20, "Graph: Error (red), Heading (green)");
  Brain.Screen.printAt(20, 150, "F: %.2f | CR: %.2f | TA: %.2f", final, inertial1.rotation(degrees), target_angle);


  int t = 0; // time counter for x-axis

  double dt = 20;

  int stableCycles = 0;

  while ( stableCycles < 5 ) {
    double current_angle = inertial1.rotation(degrees);
    double error = final - current_angle;

    double output = error * inertial_turn_kp;

    if (output > max_speed) output = max_speed;
    if (output < -max_speed) output = -max_speed;

        // --- Graph plotting ---
    // Scale values to fit screen (0–240 for y-axis, x-axis time steps)
    int x = t;
    int y_error = 120 -error;      // center error around midline
    int y_heading = 120 -current_angle;

    Brain.Screen.setPenColor(red);
    Brain.Screen.drawPixel(x, y_error);

    Brain.Screen.setPenColor(green);
    Brain.Screen.drawPixel(x, y_heading);

    t += 1;

    left_motors.spin((output >= 0) ? forward : reverse, fabs(output), percent);
    right_motors.spin((output >= 0) ? reverse : forward, fabs(output), percent);

    if (fabs(final - inertial1.rotation(degrees)) > 2.0) {
      stableCycles = 0;
    } else {
      stableCycles++;
    }

    task::sleep(dt);
  }

  left_motors.stop(brake);
  right_motors.stop(brake);
}

int drivePID() {
  double maxDriveSpeed = 50; 

  double kP_drive = 0.3;
  double kD_drive = 0.01;

  
  double leftError, rightError;
  double leftDerivative, rightDerivative;
  double leftPrevError = 0, rightPrevError = 0;

  while (true) {

    double leftTarget  = controller_1.Axis3.position(percent); 
    double rightTarget = controller_1.Axis2.position(percent);  

    leftError  = leftTarget  - left_motors.velocity(percent);
    rightError = rightTarget - right_motors.velocity(percent);

    leftDerivative  = leftError  - leftPrevError;
    rightDerivative = rightError - rightPrevError;

    double leftOutput  = (kP_drive) * leftError + kD_drive * leftDerivative;
    double rightOutput = (kP_drive) * rightError + kD_drive * rightDerivative;

    if (leftOutput > maxDriveSpeed) leftOutput = maxDriveSpeed;
    if (leftOutput < -maxDriveSpeed) leftOutput = -maxDriveSpeed;
    if (rightOutput > maxDriveSpeed) rightOutput = maxDriveSpeed;
    if (rightOutput < -maxDriveSpeed) rightOutput = -maxDriveSpeed;

    left_motors.spin(forward, leftOutput, percent);
    right_motors.spin(forward, rightOutput, percent);

    leftPrevError  = leftError;
    rightPrevError = rightError;

    task::sleep(20);  // 20 ms loop time
  }
  return 0;
}

void moveTowerMotors(int speed = 50) {
  frontAndBackMotors.setVelocity(speed, percent);
  frontBottom.setVelocity(-speed, percent);
  frontMiddle.setVelocity(-speed, percent);
  backMiddle.setVelocity(-speed, percent);
  frontAndBackMotors.spin(forward);
  frontBottom.spin(forward);
  frontMiddle.spin(forward);
  backMiddle.spin(forward);
}

void floorToBasket(int speed = 50) {
  frontBottom.setVelocity(speed, percentUnits::pct);
  frontMiddle.setVelocity(speed, percentUnits::pct);
  backMiddle.setVelocity(speed, percentUnits::pct);
  backBottom.setVelocity(speed, percentUnits::pct);
  frontTop.setVelocity(speed, percentUnits::pct);
  backTop.setVelocity(speed, percentUnits::pct);
  
  frontBottom.spin(reverse);
  frontMiddle.spin(reverse);
  backMiddle.spin(reverse);
  backBottom.spin(forward);
  frontTop.spin(forward);
  backTop.spin(reverse);
}

void printInertialInfo() {
  Brain.Screen.clearScreen();
  Brain.Screen.setFont(mono20);
  Brain.Screen.setPenColor(white);
  Brain.Screen.setFillColor(black);

  while (true) {
    Brain.Screen.printAt(10, 20, "Inertial Sensor Info:");
    Brain.Screen.printAt(10, 50, "Heading: %.2f", inertial1.heading());
    Brain.Screen.printAt(10, 80, "Rotation: %.2f", inertial1.rotation());
    Brain.Screen.printAt(10, 110, "Pitch: %.2f", inertial1.pitch());
    Brain.Screen.printAt(10, 140, "Roll: %.2f", inertial1.roll());
    Brain.Screen.printAt(10, 170, "Yaw: %.2f", inertial1.yaw());


    task::sleep(500); // Update every 500 ms
    Brain.Screen.clearScreen();
  }
}

void lowGoalScore(int speed = 50) {
  frontBottom.setVelocity(1.5*speed, percentUnits::pct);
  frontMiddle.setVelocity(speed, percentUnits::pct);
  backMiddle.setVelocity(speed, percentUnits::pct);
  backBottom.setVelocity(speed, percentUnits::pct);
  frontTop.setVelocity(speed, percentUnits::pct);
  backTop.setVelocity(speed, percentUnits::pct);
  
  frontBottom.spin(forward);
  frontMiddle.spin(forward);
  backMiddle.spin(reverse);
  backBottom.spin(reverse);
  // frontTop.spin(reverse);
  backTop.spin(reverse);
}

void middleGoalScore(int speed = 50) {
  frontBottom.setVelocity(speed, percentUnits::pct);
  frontBottom.setVelocity(speed, percentUnits::pct);
  backMiddle.setVelocity(speed, percentUnits::pct);
  backBottom.setVelocity(speed, percentUnits::pct);
  frontTop.setVelocity(speed, percentUnits::pct);
  backTop.setVelocity(speed, percentUnits::pct);
  
  frontBottom.spin(reverse);
  frontMiddle.spin(reverse);
  backMiddle.spin(reverse);
  backBottom.spin(reverse);
  frontTop.spin(reverse);
  backTop.spin(reverse);
}

void highGoalScore(int speed = 50) {
  frontBottom.setVelocity(speed, percentUnits::pct);
  frontMiddle.setVelocity(speed, percentUnits::pct);
  backMiddle.setVelocity(speed, percentUnits::pct);
  backBottom.setVelocity(speed, percentUnits::pct);
  frontTop.setVelocity(speed, percentUnits::pct);
  backTop.setVelocity(speed, percentUnits::pct);
  
  frontBottom.spin(reverse);
  frontMiddle.spin(reverse);
  backMiddle.spin(reverse);
  backBottom.spin(reverse);
  frontTop.spin(forward);
  backTop.spin(forward);
}
void shootOutBalls(int speed = 50) {
  frontBottom.setVelocity(0.5*speed, percentUnits::pct);
  frontMiddle.setVelocity(0.5*speed, percentUnits::pct);
  frontTop.setVelocity(speed, percentUnits::pct);

  frontBottom.spin(reverse);
  frontMiddle.spin(reverse);
  frontTop.spin(reverse);

}

// Button areas (x1, y1, x2, y2)
int redButton[4]  = {20, 40, 200, 120};   // left button
int blueButton[4] = {220, 40, 400, 120};  // right button

// Draw buttons
void drawButtons() {
  Brain.Screen.clearScreen();

  // Red button
  Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(redButton[0], redButton[1],
                             redButton[2]-redButton[0],
                             redButton[3]-redButton[1]);
  Brain.Screen.setPenColor(white);
  Brain.Screen.printAt(redButton[0]+50, redButton[1]+50, "RED");

  // Blue button
  Brain.Screen.setFillColor(blue);
  Brain.Screen.drawRectangle(blueButton[0], blueButton[1],
                             blueButton[2]-blueButton[0],
                             blueButton[3]-blueButton[1]);
  Brain.Screen.setPenColor(white);
  Brain.Screen.printAt(blueButton[0]+50, blueButton[1]+50, "BLUE");
}

// Check if point inside rectangle
bool isInside(int x, int y, int rect[4]) {
  return (x >= rect[0] && x <= rect[2] &&
          y >= rect[1] && y <= rect[3]);
}


void setTowerMode() {
  if (towerMode != MODE_INTAKE && towerMode != MODE_SHOOT_OUT_BALLS) {
    return;
  }
  
  const int TIME_UNTIL_CONJUNCTION = 280;
  const int TIME_UNTIL_EJECTION = 365;

  int currentTime = Brain.Timer.time(msec);

  isNearObject = colorSensor.isNearObject();

  int hue = colorSensor.hue();

  observedColor = ( hue > 120) ? BLUE : ( hue < 23) ? RED : NONE;

  if (observedColor != prevObservedColor && prevObservedColor != NONE) {
    if (intakeq.empty() && ejectq.empty()) {
      towerMode = prevObservedColor == myTeam ? MODE_INTAKE : MODE_SHOOT_OUT_BALLS;

    }
    intakeq.push_front(Ball(prevObservedColor, currentTime));
  }
  if (!intakeq.empty()) {
    Ball backOfIntakeq = intakeq.back();
    if (currentTime - backOfIntakeq.getObservationTime() > TIME_UNTIL_CONJUNCTION) {

      towerMode = backOfIntakeq.getColor() == myTeam ? MODE_INTAKE : MODE_SHOOT_OUT_BALLS;

      ejectq.push_front(backOfIntakeq);
      intakeq.pop_back();
    }
  }
  if (!ejectq.empty()) {
    Ball backOfEjectq = ejectq.back();
    if (currentTime - backOfEjectq.getObservationTime() > TIME_UNTIL_EJECTION) {

      ejectq.pop_back();
    }
  }
  isPrevNearObject = isNearObject;
  prevObservedColor = observedColor;
}

void setTowerMotors() {
  switch(towerMode) {
    case MODE_OFF:
      frontAndBackMotors.stop();
      frontBottom.stop();
      frontMiddle.stop();
      backMiddle.stop();
      break;
    case MODE_SCORE_BOTTOM:
      lowGoalScore(50);
      break;
    case MODE_SCORE_MIDDLE:
      middleGoalScore(50);
      break;
    case MODE_SCORE_TOP:
      highGoalScore(50);
      break;
    case MODE_INTAKE:
      floorToBasket(50);
      break;
    case MODE_SHOOT_OUT_BALLS:
      shootOutBalls(100);
      break;
  }
}


int manual_drive() {

  drawButtons();

  while (myTeam == NONE) {
    if (Brain.Screen.pressing()) {
      int x = Brain.Screen.xPosition();
      int y = Brain.Screen.yPosition();

      if (isInside(x, y, redButton)) {
        myTeam = RED;
        Brain.Screen.clearScreen();
        drawButtons();
        Brain.Screen.printAt(100, 200, "Red selected!");
      }
      else if (isInside(x, y, blueButton)) {
        myTeam = BLUE;
        Brain.Screen.clearScreen();
        drawButtons();
        Brain.Screen.printAt(100, 200, "Blue selected!");
      }

      // simple debounce delay
      this_thread::sleep_for(300);
    }
    this_thread::sleep_for(20);
  }
  while (true) {

    if (controller_1.ButtonR2.pressing()) {
      towerMode = MODE_SCORE_BOTTOM;
    } else if (controller_1.ButtonR1.pressing()) {
      towerMode = MODE_SCORE_MIDDLE;
    } else if (controller_1.ButtonL1.pressing()) {
      towerMode = MODE_SCORE_TOP;
    } else if (controller_1.ButtonL2.pressing()) {
      towerMode = MODE_INTAKE;
    } else if (controller_1.ButtonA.pressing()) {
      towerMode = MODE_OFF;
    }

    setTowerMotors();

    user_control();

    setTowerMode();

    task::sleep(40);

  }
  return 0;
}

int auton() {
  inertial1.calibrate();

  while (inertial1.isCalibrating()) {
    task::sleep(100);
  }

  inertial1.setRotation(0, degrees);


  straight(24);
  inertial_turn(-30);

  towerMode = MODE_INTAKE;
  setTowerMotors();
  
  straight(10);
  straight(-10);

  towerMode = MODE_OFF;
  setTowerMotors();
  
  inertial_turn(75);
  straight(17);

  return 0;
}

int main() {
  //return manual_drive();
  return auton();
}


