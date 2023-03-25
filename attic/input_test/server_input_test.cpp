
#include <iostream>
#include <string>
#include <winsock2.h>
#include <windows.h>

// WinSock2
#pragma comment(lib, "ws2_32.lib")

#include "util/u_json.hpp"

using namespace xrt::auxiliary::util::json;

struct fake_emulated_buttons_state
{
    bool a;
    bool b;
    bool trigger;
    bool grip; // Not emulated, but you can get at it through the UI

    bool system; // ditto

    uint8_t _pad[2];

    bool thumbstick_gesture;

    float thumbstick_x;
    float thumbstick_y; // Not emulated, but you can get at it through UI

    //
    float curls[5];
};

void blah(const char *stuff)
{
    JSONNode node(stuff);

    if (node.isInvalid())
    {
        printf("Not valid JSON\n");
        return;
    }

    if (!node.isObject())
    {
        printf("Valid JSON, but not an object ({})\n");
        return;
    }

    std::string names[2] = {"Left", "Right"};

    for (int i = 0; i < 2; i++)
    {
        JSONNode side = node[names[i]];
        bool a = side["a"].asInt();
        bool b = side["b"].asInt();

        // defaults
        bool trigger = side["trigger"].asInt();
        bool grip = side["grip"].asInt();

        bool thumbstick_active = side["thumbstick_active"].asBool();

        float thumbstick_x = side["thumbstick_x"].asDouble();
        float thumbstick_y = side["thumbstick_y"].asDouble();

        printf("%s: %d %d %d %d  %d %f %f\n", names[i].c_str(), a, b, trigger, grip, thumbstick_active, thumbstick_x, thumbstick_y);
    }
}

int main()
{
    sockaddr_in localAddr;
    SOCKET clientSocket;
    SOCKET listenSocket;

    // Initialize Winsock
    WSADATA wsaData;
    int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != 0)
    {
        printf("WSAStartup failed: %d", iResult);
        return 1;
    }

    // Create a socket for the subprocess to connect to
    listenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listenSocket == INVALID_SOCKET)
    {
        printf("Error creating socket: %d", WSAGetLastError());
        WSACleanup();
        return 1;
    }

    // Bind the socket to any available address and port 0 to let the operating system choose a free port
    sockaddr_in listenAddr;
    listenAddr.sin_family = AF_INET;
    listenAddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    listenAddr.sin_port = htons(12428);
    iResult = bind(listenSocket, (sockaddr *)&listenAddr, sizeof(listenAddr));
    if (iResult == SOCKET_ERROR)
    {
        printf("Error binding socket: %d", WSAGetLastError());
        closesocket(listenSocket);
        WSACleanup();
        return 1;
    }

    // Get the local address and port of the socket
    // sockaddr_in localAddr;
    int localAddrLen = sizeof(localAddr);
    iResult = getsockname(listenSocket, (sockaddr *)&localAddr, &localAddrLen);
    if (iResult == SOCKET_ERROR)
    {
        printf("Error getting socket name: %d", WSAGetLastError());
        closesocket(listenSocket);
        WSACleanup();
        return 1;
    }

    // SetupListen

    printf("Server: Listening!\n");

    // Listen for the subprocess to connect
    iResult = listen(listenSocket, SOMAXCONN);
    if (iResult == SOCKET_ERROR)
    {
        printf("Error listening for connection: %d", WSAGetLastError());
        closesocket(listenSocket);
        WSACleanup();
        return false;
    }
    printf("Server: Accepting connection!\n");

    // Accept the connection
    clientSocket = accept(listenSocket, NULL, NULL);
    if (clientSocket == INVALID_SOCKET)
    {
        printf("Error accepting connection: %d", WSAGetLastError());
        closesocket(listenSocket);
        WSACleanup();
        return false;
    }

    // end SetupListen

    while (true)
    {

        int iResult = 0;
        char message[2048] = {};
        iResult = recv(clientSocket, message, 2047, 0);

        blah(message);
        printf("Got message %s\n\n\n\n", message);
    }

    return 0;
}