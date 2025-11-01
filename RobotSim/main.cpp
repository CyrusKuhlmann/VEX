#include <winsock2.h>
#include <ws2tcpip.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#pragma comment(lib, "ws2_32.lib")  // Visual Studio only

#define M_PI 3.14159265358979323846

const double TRACKING_WHEEL_DIAMETER = 2;
const double TRACKING_WHEEL_CIRCUMFERENCE = TRACKING_WHEEL_DIAMETER * M_PI;
const double DRIVE_WHEEL_DIAMETER = 4.0;
const double DRIVE_WHEEL_CIRCUMFERENCE = DRIVE_WHEEL_DIAMETER * M_PI;
const double TL = 7.25;  // distance from left wheel to center
const double TR = 7.25;  // distance from right wheel to center
const double TB = 7.75;  // distance from back wheel to center

class Robot {
 private:
  std::string ip_address;
  int port_number;
  SOCKET sock;
  bool connected;
  double rightSensorRotations;
  double leftSensorRotations;
  double backSensorRotations;
  double inertialHeading;
  double robotX;
  double robotY;
  double robotTheta;
  double prevLeft;
  double prevRight;
  double prevBack;
  double clock;

  bool connect_to_server() {
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
      std::cerr << "WSAStartup failed." << std::endl;
      return false;
    }

    sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock == INVALID_SOCKET) {
      std::cerr << "Socket creation failed." << std::endl;
      WSACleanup();
      return false;
    }

    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port_number);
    inet_pton(AF_INET, ip_address.c_str(), &serverAddr.sin_addr);

    if (connect(sock, (sockaddr*)&serverAddr, sizeof(serverAddr)) ==
        SOCKET_ERROR) {
      std::cerr << "Connection to robot failed." << std::endl;
      closesocket(sock);
      WSACleanup();
      return false;
    }

    connected = true;
    std::cout << "Connected to robot simulator at " << ip_address << ":"
              << port_number << std::endl;
    return true;
  }

  void sendMsg(std::string command) {
    if (!connected) {
      std::cerr << "Not connected to robot." << std::endl;
      return;
    }

    std::string msg = command;
    int result = send(sock, msg.c_str(), msg.size(), 0);
    if (result == SOCKET_ERROR) {
      std::cerr << "Send failed." << std::endl;
      return;
    }

    char buffer[1024] = {0};
    int bytesReceived = recv(sock, buffer, 1024, 0);
    if (bytesReceived > 0) {
      std::string data = std::string(buffer, bytesReceived);
      std::stringstream ss(data);
      std::vector<double> numbers;
      std::string temp;

      while (std::getline(ss, temp, '|')) {
        // remove whitespace
        temp.erase(remove_if(temp.begin(), temp.end(), ::isspace), temp.end());
        if (!temp.empty()) {
          numbers.push_back(std::stod(temp));
        }
      }
      if (numbers.size() >= 5) {
        clock = numbers[0];
        leftSensorRotations = numbers[1];
        rightSensorRotations = numbers[2];
        backSensorRotations = numbers[3];
        inertialHeading = numbers[4];
      }
    } else if (bytesReceived == 0) {
      std::cout << "Connection closed by server." << std::endl;
      connected = false;
    } else {
      std::cerr << "Receive failed." << std::endl;
    }
  }

 public:
  Robot(const std::string& ip, int port)
      : ip_address(ip),
        port_number(port),
        connected(false),
        clock(0.0),
        rightSensorRotations(0.0),
        leftSensorRotations(0.0),
        backSensorRotations(0.0),
        inertialHeading(0.0),
        robotX(0.0),
        robotY(0.0),
        robotTheta(0.0),
        prevLeft(0.0),
        prevRight(0.0),
        prevBack(0.0) {
    connect_to_server();
  }

  ~Robot() {
    if (connected) {
      closesocket(sock);
      WSACleanup();
    }
  }

  double getInertialHeading() { return inertialHeading; }
  double getLeftSensorRotation() { return leftSensorRotations; }
  double getRightSensorRotation() { return rightSensorRotations; }
  double getBackSensorRotation() { return backSensorRotations; }
  double getRobotX() { return 0.0; }
  double getRobotY() { return 0.0; }
  double getRobotTheta() { return 0.0; }

  void sleepFor(int milliseconds) {
    double start_time = clock;
    while ((clock - start_time) * 1000.0 < milliseconds) {
      sendMsg("get_sensors");
    }
  }

  void getSensors() { sendMsg("get_sensors"); }

  void setLeftMotorVelocity(double leftVelocity) {
    sendMsg("set_velocity | left | " + std::to_string(leftVelocity));
  }

  void setRightMotorVelocity(double rightVelocity) {
    sendMsg("set_velocity | right | " + std::to_string(rightVelocity));
  }

  void setLeftMotorSpin(std::string leftSpin) {
    sendMsg("spin_motor | left | " + leftSpin);
  }

  void setRightMotorSpin(std::string rightSpin) {
    sendMsg("spin_motor | right | " + rightSpin);
  }

  void setLeftMotorStop() { sendMsg("stop_motor | left"); }

  void setRightMotorStop() { sendMsg("stop_motor | right"); }

  void odom() {
    double left = leftSensorRotations * TRACKING_WHEEL_CIRCUMFERENCE;
    double right = rightSensorRotations * TRACKING_WHEEL_CIRCUMFERENCE;
    double back = backSensorRotations * TRACKING_WHEEL_CIRCUMFERENCE;

    double deltaLeft = left - prevLeft;
    double deltaRight = right - prevRight;
    double deltaBack = back - prevBack;

    double deltaTheta = (deltaLeft - deltaRight) / (TL + TR);
    double prevTheta = robotTheta;

    double deltaS = (deltaLeft + deltaRight) / 2.0;
    deltaTheta = (deltaRight - deltaLeft) / (TL + TR);
    robotTheta += deltaTheta;

    double deltaBack_corrected = deltaBack - TB * deltaTheta;

    double sinDiff = sin(robotTheta) - sin(prevTheta);
    double cosDiff = cos(robotTheta) - cos(prevTheta);

    if (fabs(deltaTheta) > 1e-6) {
      double radius = deltaS / deltaTheta;
      robotX += radius * sinDiff + deltaBack_corrected * cosDiff;
      robotY += -radius * cosDiff + deltaBack_corrected * sinDiff;
    } else {
      robotX += deltaS * cos(prevTheta) - deltaBack_corrected * sin(prevTheta);
      robotY += deltaS * sin(prevTheta) + deltaBack_corrected * cos(prevTheta);
    }

    prevLeft = left;
    prevRight = right;
    prevBack = back;
  }

  void turn_to_angle(double target_angle, double max_speed) {
    double Kp_angle = 0.250;
    double Ki_angle = 0.007;
    double Kd_angle = 0.029;

    double Kp_drift = 0.210;
    double Kd_drift = 0.035;

    double angle_error_prev = 0.0;
    double angle_integral = 0.0;

    double drift_error_prev = 0.0;

    double output = 0.0;
    double slew_rate = 2.0;

    int stable_count = 0;
    const int required_stable_cycles = 5;
    const double tolerance = 3;
    const double dt = 0.02;

    while (stable_count < required_stable_cycles) {
      double left = leftSensorRotations * TRACKING_WHEEL_CIRCUMFERENCE;
      double right = rightSensorRotations * TRACKING_WHEEL_CIRCUMFERENCE;

      double current_angle = robotTheta * (180.0 / M_PI);

      double angle_error = target_angle - current_angle;
      angle_integral += angle_error * dt;
      double angle_derivative = (angle_error - angle_error_prev) / dt;
      angle_error_prev = angle_error;

      double angle_output = Kp_angle * angle_error + Ki_angle * angle_integral +
                            Kd_angle * angle_derivative;

      if (angle_output > output + slew_rate)
        output += slew_rate;
      else if (angle_output < output - slew_rate)
        output -= slew_rate;
      else
        output = angle_output;

      if (output > max_speed) output = max_speed;
      if (output < -max_speed) output = -max_speed;

      double drift_error = fabs(left) - fabs(right);
      double drift_derivative = (drift_error - drift_error_prev) / dt;
      drift_error_prev = drift_error;

      double drift_correction =
          Kp_drift * drift_error + Kd_drift * drift_derivative;

      double left_speed = output + drift_correction;
      double right_speed = -output - drift_correction;

      if (left_speed > max_speed) left_speed = max_speed;
      if (right_speed > max_speed) right_speed = max_speed;

      if (left_speed < -max_speed) left_speed = -max_speed;
      if (right_speed < -max_speed) right_speed = -max_speed;

      setLeftMotorVelocity(fabs(left_speed));
      setRightMotorVelocity(fabs(right_speed));
      setLeftMotorSpin(left_speed >= 0 ? "forward" : "reverse");
      setRightMotorSpin(right_speed >= 0 ? "forward" : "reverse");

      if (fabs(angle_error) < tolerance) {
        stable_count++;
      } else {
        stable_count = 0;
      }

      sleepFor(20);
      odom();
      // print debug info
      {
        std::cout << " | Angle Error: " << angle_error << std::endl;
      }

      setLeftMotorStop();
      setRightMotorStop();
    }
  }
};

int main() {
  Robot myRobot("127.0.0.1", 65432);

  myRobot.turn_to_angle(90.0, 30.0);

  while (true) {
    myRobot.sleepFor(20);
  }

  return 0;
}
