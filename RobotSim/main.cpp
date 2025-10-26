#include <winsock2.h>
#include <ws2tcpip.h>

#include <iostream>
#include <string>

#pragma comment(lib, "ws2_32.lib")  // Visual Studio only

int main() {
  WSADATA wsaData;
  if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
    std::cerr << "WSAStartup failed\n";
    return 1;
  }

  SOCKET sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock == INVALID_SOCKET) {
    std::cerr << "Socket creation failed\n";
    WSACleanup();
    return 1;
  }

  sockaddr_in serv_addr;
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(65432);
  inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr);

  if (connect(sock, (sockaddr*)&serv_addr, sizeof(serv_addr)) == SOCKET_ERROR) {
    std::cerr << "Connect failed\n";
    closesocket(sock);
    WSACleanup();
    return 1;
  }

  std::string msg = "MOVE 1 0.5";
  send(sock, msg.c_str(), msg.size(), 0);

  char buffer[1024] = {0};
  int bytesReceived = recv(sock, buffer, 1024, 0);
  if (bytesReceived > 0)
    std::cout << "Robot state: " << std::string(buffer, bytesReceived)
              << std::endl;

  closesocket(sock);
  WSACleanup();
}
