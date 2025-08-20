/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       cyrus                                                     */
/*    Created:      8/9/2025, 9:03:31 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath>

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
double kP = 0.04;
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
bool doTurnPID = false;

bool resetDriveSensors = false;
bool resetTurnSensors = false;



void drivePID(int targetPosition) {

  // Exit conditions
  double tolerance = 1.0;          // degrees
  int stableTimeRequired = 500;    // ms
  int stableTime = 0;
  int loopDelay = 20;               // ms
  int timeout = 3000;               // ms
  int elapsedTime = 0;

  while (elapsedTime < timeout) {

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
    derivative = (error - prevError) / (loopDelay / 1000.0); 
    totalError += error * (loopDelay / 1000.0); 

    double lateralMotorPower = ((kP * error) + (kI * totalError) + (kD * derivative));
    ///////////////////////////////////////////////////////////////////////////////
    Left_Motors.spin(forward, lateralMotorPower, voltageUnits::volt);
    Right_Motors.spin(forward, lateralMotorPower, voltageUnits::volt);


    // --- Exit condition --- 
    if (std::abs(error) <= tolerance) {
        stableTime += loopDelay;
        if (stableTime >= stableTimeRequired) {
            break;
        }
    } 
    else {
        stableTime = 0; 
    }

    prevError = error;
    task::sleep(loopDelay); 
    elapsedTime += loopDelay;

  }

  Left_Motors.stop();
  Right_Motors.stop();

}

// void turnPID(int targetTurnPosition) {
//   while (doTurnPID) {

//     if (resetTurnSensors) {
//       Left_Motors.setPosition(0, degrees);
//       Right_Motors.setPosition(0, degrees);
//       resetTurnSensors = false;
//     }
    
//     int leftMotorPosition = Left_Motors.position(degrees);
//     int rightMotorPosition = Right_Motors.position(degrees);

//     /////////////////////////////////////////////////////////
//     // Turn PID Control
//     /////////////////////////////////////////////////////////
//     int turnDifference = leftMotorPosition - rightMotorPosition;

//     // proportional, derivative, and integral
//     turnError = targetTurnPosition - turnDifference;
//     turnDerivative = turnError - turnPrevError;
//     turnTotalError += turnError;

//     double turnMotorPower = ((turnkP * turnError) + (turnkI * turnTotalError) + (turnkD * turnDerivative));

//     Left_Motors.spin(forward, turnMotorPower, voltageUnits::volt);
//     Right_Motors.spin(forward, -turnMotorPower, voltageUnits::volt);

//     turnPrevError = turnError;
//     task::sleep(loopDelay);

//   }

//   Left_Motors.stop();
//   Right_Motors.stop(); 

// }


void autonomous(void) {
  drivePID(300);
  drivePID(-300); 
}


int main() {
  autonomous();
}
