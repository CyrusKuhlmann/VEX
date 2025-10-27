#include <winsock2.h>
#include <ws2tcpip.h>

#include <iostream>
#include <string>

#pragma comment(lib, "ws2_32.lib")  // Visual Studio only

class Robot {
 private:
  std::string ip_address;
  int port_number;
  SOCKET sock;
  void sendMsg(std::string command) {
    std::string msg = command;
    send(sock, msg.c_str(), msg.size(), 0);

    char buffer[1024] = {0};
    int bytesReceived = recv(sock, buffer, 1024, 0);
    if (bytesReceived > 0)
      std::cout << "Robot state: " << std::string(buffer, bytesReceived)
                << std::endl;

    closesocket(sock);
    WSACleanup();
  };

 public:
  Robot(const std::string& ip, int port) : ip_address(ip), port_number(port) {
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);

    sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock == INVALID_SOCKET) {
      std::cerr << "Socket creation failed." << std::endl;
      return;
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
      return;
    }
  }
  double getInertialHeading() { return 0.0; };
  double getLeftMotorRotation() { return 0.0; };
  double getRightMotorRotation() { return 0.0; };
  double getParallelSensorRotation() { return 0.0; };
  double getPerpendicularSensorRotation() { return 0.0; };
  double getRobotX() { return 0.0; };
  double getRobotY() { return 0.0; };
  double getRobotTheta() { return 0.0; };

  void setLeftMotorVelocity(double leftVelocity) {
    sendMsg("set_velocity | left | " + std::to_string(leftVelocity));
  };
  void setRightMotorVelocity(double rightVelocity) {
    sendMsg("set_velocity | right | " + std::to_string(rightVelocity));
  };

  void setLeftMotorSpin(double leftSpin) {
    sendMsg("set_turn_state | left | " + std::to_string(leftSpin));
  };
  void setRightMotorSpin(double rightSpin) {
    sendMsg("set_turn_state | right | " + std::to_string(rightSpin));
  };
  void setLeftMotorStop(double leftStop) {
    sendMsg("set_stop | left | " + std::to_string(leftStop));
  };
  void setRightMotorStop(double rightStop) {
    sendMsg("set_stop | right | " + std::to_string(rightStop));
  };
};

int main() {
  Robot myRobot("127.0.0.1", 65432);
  myRobot.setLeftMotorVelocity(100);
  myRobot.setRightMotorVelocity(100);
  return 0;
}
