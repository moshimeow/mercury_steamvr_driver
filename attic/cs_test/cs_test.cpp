#include <iostream>
#include <string>
#include <winsock2.h>
#include <windows.h>
#include "cs_test_common.hpp"
#include "stdio.h"

#pragma comment(lib, "ws2_32.lib")

int main()
{
    // Initialize Winsock
    WSADATA wsaData;
    int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != 0)
    {
        std::cerr << "WSAStartup failed: " << iResult << std::endl;
        return 1;
    }

    // Create a socket for the subprocess to connect to
    SOCKET listenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listenSocket == INVALID_SOCKET)
    {
        std::cerr << "Error creating socket: " << WSAGetLastError() << std::endl;
        WSACleanup();
        return 1;
    }

    // Bind the socket to any available address and port 0 to let the operating system choose a free port
    sockaddr_in listenAddr;
    listenAddr.sin_family = AF_INET;
    listenAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    listenAddr.sin_port = htons(0);
    iResult = bind(listenSocket, (sockaddr *)&listenAddr, sizeof(listenAddr));
    if (iResult == SOCKET_ERROR)
    {
        std::cerr << "Error binding socket: " << WSAGetLastError() << std::endl;
        closesocket(listenSocket);
        WSACleanup();
        return 1;
    }

    // Get the local address and port of the socket
    sockaddr_in localAddr;
    int localAddrLen = sizeof(localAddr);
    iResult = getsockname(listenSocket, (sockaddr *)&localAddr, &localAddrLen);
    if (iResult == SOCKET_ERROR)
    {
        std::cerr << "Error getting socket name: " << WSAGetLastError() << std::endl;
        closesocket(listenSocket);
        WSACleanup();
        return 1;
    }

    // Start the subprocess with the local address and port as arguments
    STARTUPINFO startupInfo;
    PROCESS_INFORMATION processInfo;
    ZeroMemory(&startupInfo, sizeof(startupInfo));
    ZeroMemory(&processInfo, sizeof(processInfo));
    startupInfo.cb = sizeof(startupInfo);
    startupInfo.wShowWindow = true;
    std::string commandLine = "C:\\dev\\mercury_steamvr_driver\\build\\attic\\cs_test\\cs_test_subprocess.exe " + std::to_string(ntohs(localAddr.sin_port));
    std::wstring wideCommandLine(commandLine.begin(), commandLine.end());
    if (!CreateProcess(NULL, commandLine.data(), NULL, NULL, FALSE, 0, NULL, NULL, &startupInfo, &processInfo))
    {
        std::cerr << "Error creating subprocess: " << GetLastError() << std::endl;
        closesocket(listenSocket);
        WSACleanup();
        return 1;
    }
    std::cout << "Server: Listening!\n";

    // Listen for the subprocess to connect
    iResult = listen(listenSocket, SOMAXCONN);
    if (iResult == SOCKET_ERROR)
    {
        std::cerr << "Error listening for connection: " << WSAGetLastError() << std::endl;
        closesocket(listenSocket);
        WSACleanup();
        return 1;
    }
    std::cout << "Server: Accepting connection!\n";

    // Accept the connection
    SOCKET clientSocket = accept(listenSocket, NULL, NULL);
    if (clientSocket == INVALID_SOCKET)
    {
        std::cerr << "Error accepting connection: " << WSAGetLastError() << std::endl;
        closesocket(listenSocket);
        WSACleanup();
        return 1;
    }

    std::cout << "Server: Accepted connection!\n";

    while (true)
    {
        // Receive data from the subprocess
        tracking_message message = {};
        iResult = recv(clientSocket, (char *)&message, TMSIZE, MSG_WAITALL);
        if (iResult == SOCKET_ERROR)
        {
            std::cerr << "Error receiving data: " << WSAGetLastError() << std::endl;
            closesocket(clientSocket);
            closesocket(listenSocket);
            WSACleanup();
            return 1;
        }
        // printf("Received! Size: %zu, timestamp", message.size, message.timestamp)
        printf("Received! ");
        // printf("Size: %zu, timestamp %zu, wrist %f %f %f, %f %f %f %f", message.size, message.timestamp,
        //        message.hands[0].wrist.position.x, message.hands[0].wrist.position.y, message.hands[0].wrist.position.z message.hands[0].wrist.orientation.w, message.hands[0].wrist.orientation.x, message.hands[0].wrist.orientation.y, message.hands[0].wrist.orientation.z);
        printf(TM_FMT(message));
        // std::cout << "Received!";
        // recvBuffer[iResult] = '\0';
        // std::cout << "Received data from subprocess: " << recvBuffer << std::endl;
    }
    // Send data to the subprocess
    const char *sendBuffer = "Hello, subprocess!";
    iResult = send(clientSocket, sendBuffer, strlen(sendBuffer), 0);
    if (iResult == SOCKET_ERROR)
    {
        std::cerr << "Error sending data: " << WSAGetLastError() << std::endl;
        closesocket(clientSocket);
        closesocket(listenSocket);
        WSACleanup();
        return 1;
    }

    // Close the sockets and cleanup Winsock
    closesocket(clientSocket);
    closesocket(listenSocket);
    WSACleanup();

    return 0;
}
