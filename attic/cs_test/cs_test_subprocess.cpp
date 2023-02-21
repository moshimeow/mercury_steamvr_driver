#include <iostream>
#include <string>
#include <winsock2.h>
#include <windows.h>

#pragma comment(lib, "ws2_32.lib")
#include <winsock2.h>
#include <iostream>

int main(int argc, char **argv) {
    // Initialize Winsock
    WSADATA wsaData;
    int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != 0) {
        std::cerr << "WSAStartup failed: " << iResult << std::endl;
        return 1;
    }

    // Create a socket to connect to the parent process
    SOCKET connectSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (connectSocket == INVALID_SOCKET) {
        std::cerr << "Error creating socket: " << WSAGetLastError() << std::endl;
        WSACleanup();
        return 1;
    }

    // Connect to the parent process
    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
        if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <port>" << std::endl;
        return 1;
    }
    serverAddr.sin_port = htons(atoi(argv[1]));
    iResult = connect(connectSocket, reinterpret_cast<SOCKADDR*>(&serverAddr), sizeof(serverAddr));
    if (iResult == SOCKET_ERROR) {
        std::cerr << "Error connecting to server: " << WSAGetLastError() << std::endl;
        closesocket(connectSocket);
        WSACleanup();
        return 1;
    }

    // Send data to the parent process
    const char* sendBuffer = "Hello, parent process!";
    iResult = send(connectSocket, sendBuffer, strlen(sendBuffer), 0);
    if (iResult == SOCKET_ERROR) {
        std::cerr << "Error sending data: " << WSAGetLastError() << std::endl;
        closesocket(connectSocket);
        WSACleanup();
        return 1;
    }

    // Receive data from the parent process
    char recvBuffer[1024];
    iResult = recv(connectSocket, recvBuffer, sizeof(recvBuffer), 0);
    if (iResult == SOCKET_ERROR) {
        std::cerr << "Error receiving data: " << WSAGetLastError() << std::endl;
        closesocket(connectSocket);
        WSACleanup();
        return 1;
    }
    recvBuffer[iResult] = '\0';
    std::cout << "Received data from parent process: " << recvBuffer << std::endl;

    // Close the socket and cleanup Winsock
    closesocket(connectSocket);
    WSACleanup();

    return 0;
}