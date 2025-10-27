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
  bool connected;

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
      std::cout << "Robot state: " << std::string(buffer, bytesReceived)
                << std::endl;
    } else if (bytesReceived == 0) {
      std::cout << "Connection closed by server." << std::endl;
      connected = false;
    } else {
      std::cerr << "Receive failed." << std::endl;
    }
  }

 public:
  Robot(const std::string& ip, int port)
      : ip_address(ip), port_number(port), connected(false) {
    connect_to_server();
  }

  ~Robot() {
    if (connected) {
      closesocket(sock);
      WSACleanup();
    }
  }

  double getInertialHeading() { return 0.0; }
  double getLeftMotorRotation() { return 0.0; }
  double getRightMotorRotation() { return 0.0; }
  double getParallelSensorRotation() { return 0.0; }
  double getPerpendicularSensorRotation() { return 0.0; }
  double getRobotX() { return 0.0; }
  double getRobotY() { return 0.0; }
  double getRobotTheta() { return 0.0; }

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
};

int main() {
  Robot myRobot("127.0.0.1", 65432);

  // Test sequence
  std::cout << "Setting motor velocities..." << std::endl;
  myRobot.setRightMotorVelocity(100);
  myRobot.setLeftMotorVelocity(100);

  myRobot.setRightMotorSpin("forward");
  myRobot.setLeftMotorSpin("forward");

  return 0;
}
