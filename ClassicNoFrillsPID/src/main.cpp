/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       cyrus                                                     */
/*    Created:      8/9/2025, 9:03:31 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

brain Brain;
motor leftMotorA(PORT1, ratio18_1, false);
motor leftMotorB(PORT11, ratio18_1, false);
motor_group Left_Motors(leftMotorA, leftMotorB);

motor rightMotorA(PORT10, ratio18_1, true);
motor rightMotorB(PORT20, ratio18_1, true);
motor_group Right_Motors(rightMotorA, rightMotorB);

controller controller_1(primary);

//settings
double kP = 0.08;
double kI = 0.0;
double kD = 0.0;

double turnkP = 0.08;
double turnkI = 0.0;
double turnkD = 0.0;

int error = 0;
int prevError = 0;
int derivative;
int totalError = 0;

int turnError = 0;
int turnPrevError = 0;
int turnDerivative;
int turnTotalError = 0;

bool doDrivePID = false;

bool resetDriveSensors = false;

int targetPosition = 0;
int targetTurnPosition = 0;

int drivePID(void*) {
  while (doDrivePID) {

    if (resetDriveSensors) {
      Left_Motors.setPosition(0, degrees);
      Right_Motors.setPosition(0, degrees);
      resetDriveSensors = false;
    }

    int leftMotorPosition = Left_Motors.position(degrees);
    int rightMotorPosition = Right_Motors.position(degrees);

    /////////////////////////////////////////////////////////
    // Lateral PID Control
    /////////////////////////////////////////////////////////
    int averagePosition = (leftMotorPosition + rightMotorPosition) / 2;

    // proportional, derivative, and integral
    error = targetPosition - averagePosition;
    derivative = error - prevError;
    totalError += error;

    double lateralMotorPower = ((kP * error) + (kI * totalError) + (kD * derivative));
    ///////////////////////////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////
    // Turn PID Control
    /////////////////////////////////////////////////////////
    int turnDifference = leftMotorPosition - rightMotorPosition;

    // proportional, derivative, and integral
    turnError = targetTurnPosition - turnDifference;
    turnDerivative = turnError - turnPrevError;
    turnTotalError += turnError;

    double turnMotorPower = ((turnkP * turnError) + (turnkI * turnTotalError) + (turnkD * turnDerivative));
    ///////////////////////////////////////////////////////////////////////////////
    Left_Motors.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    Right_Motors.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);

    prevError = error;
    turnPrevError = turnError;
    task::sleep(20); 

  }

  Left_Motors.stop();
  Right_Motors.stop();
  return 0;

}

void autonomous(void) {
  doDrivePID = true;
  task pidTask(drivePID, nullptr, 7);

  resetDriveSensors = true;
  targetPosition = 300; 
  targetTurnPosition = 0;

  task::sleep(2000); 

  resetDriveSensors = true; 
  targetPosition = -300; 
  targetTurnPosition = 0; 

  task::sleep(2000);

  doDrivePID = false; 

}


int main() {
  autonomous();
}
