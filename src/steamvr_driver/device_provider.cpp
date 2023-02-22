#include <iostream>
#include <string>
#include <winsock2.h>
#include <windows.h>

// WinSock2
#pragma comment(lib, "ws2_32.lib")

#include "device_provider.h"
#include "util/path_utils.h"
#include "driver_log.h"
#include "vive/vive_config.h"
#include "../../monado/src/xrt/tracking/hand/mercury/hg_interface.h"
#include "tracking/t_frame_cv_mat_wrapper.hpp"
#include "os/os_time.h"
#include "oxr_sdl2_hack.h"
#include "CameraIdxGuesser.hpp"

#include "tracking_subprocess_protocol.hpp"

namespace xat = xrt::auxiliary::tracking;

vr::EVRInitError DeviceProvider::Init(vr::IVRDriverContext *pDriverContext)
{
    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

    InitDriverLog(vr::VRDriverLog());

    // initialise hand tracking
    std::string hmd_config;
    if (!GetHMDConfigPath(hmd_config))
    {
        DriverLog("Failed to find HMD config. Exiting.");
        return vr::VRInitError_Driver_CalibrationInvalid;
    }

    // Initialize Winsock
    WSADATA wsaData;
    int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != 0)
    {
        DriverLog("WSAStartup failed: %d", iResult);
        return vr::VRInitError_Driver_Failed;
    }

    // Create a socket for the subprocess to connect to
    listenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listenSocket == INVALID_SOCKET)
    {
        DriverLog("Error creating socket: %d", WSAGetLastError());
        WSACleanup();
        return vr::VRInitError_Driver_Failed;
    }

    // Bind the socket to any available address and port 0 to let the operating system choose a free port
    sockaddr_in listenAddr;
    listenAddr.sin_family = AF_INET;
    listenAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    listenAddr.sin_port = htons(0);
    iResult = bind(listenSocket, (sockaddr *)&listenAddr, sizeof(listenAddr));
    if (iResult == SOCKET_ERROR)
    {
        DriverLog("Error binding socket: %d", WSAGetLastError());
        closesocket(listenSocket);
        WSACleanup();
        return vr::VRInitError_Driver_Failed;
    }

    // Get the local address and port of the socket
    sockaddr_in localAddr;
    int localAddrLen = sizeof(localAddr);
    iResult = getsockname(listenSocket, (sockaddr *)&localAddr, &localAddrLen);
    if (iResult == SOCKET_ERROR)
    {
        DriverLog("Error getting socket name: %d", WSAGetLastError());
        closesocket(listenSocket);
        WSACleanup();
        return vr::VRInitError_Driver_Failed;
    }

    // Start the subprocess with the local address and port as arguments
    STARTUPINFO startupInfo;
    PROCESS_INFORMATION processInfo;
    ZeroMemory(&startupInfo, sizeof(startupInfo));
    ZeroMemory(&processInfo, sizeof(processInfo));
    startupInfo.cb = sizeof(startupInfo);

    // maybe this is bad?
    // startupInfo.wShowWindow = true;

    std::string subprocessPath = {};
    GetSubprocessPath(subprocessPath);
    std::string commandLine = subprocessPath + " " + std::to_string(ntohs(localAddr.sin_port)) + " " + hmd_config;
    std::wstring wideCommandLine(commandLine.begin(), commandLine.end());
    DriverLog("Creating subprocess %s!", commandLine.c_str());
    if (!CreateProcess(NULL, commandLine.data(), NULL, NULL, FALSE, 0, NULL, NULL, &startupInfo, &processInfo))
    {
        DriverLog("Error creating subprocess %s:  %d", commandLine.c_str(), WSAGetLastError());
        closesocket(listenSocket);
        WSACleanup();
        return vr::VRInitError_Driver_Failed;
    }

#if 0

    DriverLog("Server: Listening!\n");

    // Listen for the subprocess to connect
    iResult = listen(listenSocket, SOMAXCONN);
    if (iResult == SOCKET_ERROR)
    {
        DriverLog("Error listening for connection: ", WSAGetLastError());
        closesocket(listenSocket);
        WSACleanup();
        return vr::VRInitError_Driver_Failed;
    }
    DriverLog("Server: Accepting connection!\n");

    // Accept the connection
    clientSocket = accept(listenSocket, NULL, NULL);
    if (clientSocket == INVALID_SOCKET)
    {
        DriverLog("Error accepting connection: %d", WSAGetLastError());
        closesocket(listenSocket);
        WSACleanup();
        return vr::VRInitError_Driver_Failed;
    }

#endif


    xrt_pose head_in_left = XRT_POSE_IDENTITY;


    // initialise the hands
    left_hand_ = std::make_unique<MercuryHandDevice>(vr::TrackedControllerRole_LeftHand, head_in_left);
    right_hand_ = std::make_unique<MercuryHandDevice>(vr::TrackedControllerRole_RightHand, head_in_left);

    // vr::TrackedDeviceClass_GenericTracker seems to not work

    vr::VRServerDriverHost()->TrackedDeviceAdded(left_hand_->GetSerialNumber().c_str(),
                                                 vr::TrackedDeviceClass_Controller,
                                                 left_hand_.get());
    vr::VRServerDriverHost()->TrackedDeviceAdded(right_hand_->GetSerialNumber().c_str(),
                                                 vr::TrackedDeviceClass_Controller,
                                                 right_hand_.get());


    is_active_ = true;
    hand_tracking_thread_ = std::thread(&DeviceProvider::HandTrackingThread, this);

    return vr::VRInitError_None;
}

void hjs_from_tracking_message(const struct tracking_message_hand &msg, xrt_hand_joint_set &set)
{
    if (!msg.tracked)
    {
        set.is_active = false;
        return;
    }
    set.is_active = true;

    const enum xrt_space_relation_flags valid_flags_ht = (enum xrt_space_relation_flags)(
        XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
        XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT);

    // set.values.hand_joint_set_default[XRT_HAND_JOINT_WRIST].relation.pose = msg.wrist;
    // set.values.hand_joint_set_default[XRT_HAND_JOINT_WRIST].relation.relation_flags = valid_flags_ht;

    for (int i = 0; i < XRT_HAND_JOINT_COUNT; i++)
    {
        struct xrt_relation_chain xrc = {};
        xrt_space_relation tmp = {};
        m_relation_chain_push_pose(&xrc, &msg.fingers_relative[i]);
        m_relation_chain_push_pose(&xrc, &msg.wrist);
        m_relation_chain_resolve(&xrc, &tmp);

        set.values.hand_joint_set_default[i].relation = tmp;
        set.values.hand_joint_set_default[i].relation.relation_flags = valid_flags_ht;
    }
}

void DeviceProvider::HandTrackingThread()
{
#if 1
    DriverLog( "Server: Listening!\n");

    // Listen for the subprocess to connect
    int iResult = listen(listenSocket, SOMAXCONN);
    if (iResult == SOCKET_ERROR)
    {
        DriverLog( "Error listening for connection: %d", WSAGetLastError());
        closesocket(listenSocket);
        WSACleanup();
        return;
    }
    DriverLog( "Server: Accepting connection!\n");

    // Accept the connection
    clientSocket = accept(listenSocket, NULL, NULL);
    if (clientSocket == INVALID_SOCKET)
    {
        DriverLog("Error accepting connection: %d", WSAGetLastError());
        closesocket(listenSocket);
        WSACleanup();
        return;
    }
#endif

    DriverLog("HandTrackingThread!");

    while (is_active_)
    {
        DriverLog("HandTrackingThreadOne!");

        // Receive data from the subprocess
        int iResult = 0;
        tracking_message message = {};
        iResult = recv(clientSocket, (char *)&message, TMSIZE, MSG_WAITALL);
        DriverLog("Received! Result: %d", iResult);
        if (iResult == SOCKET_ERROR)
        {
            DriverLog("Error receiving data: %d", WSAGetLastError());
            closesocket(clientSocket);
            closesocket(listenSocket);
            WSACleanup();
            return;
        }
        // printf("Received! Size: %zu, timestamp", message.size, message.timestamp)

        if (iResult != TMSIZE)
        {
            DriverLog("Message was the wrong size: %d", iResult);
            continue;
        }

        // printf("Size: %zu, timestamp %zu, wrist %f %f %f, %f %f %f %f", message.size, message.timestamp,
        //        message.hands[0].wrist.position.x, message.hands[0].wrist.position.y, message.hands[0].wrist.position.z message.hands[0].wrist.orientation.w, message.hands[0].wrist.orientation.x, message.hands[0].wrist.orientation.y, message.hands[0].wrist.orientation.z);
        DriverLog(TM_FMT(message));

        xrt_hand_joint_set hands[2] = {};

        DriverLog("Processing hand joint sets \n");

        hjs_from_tracking_message(message.hands[0], hands[0]);
        hjs_from_tracking_message(message.hands[1], hands[1]);

        DriverLog("Updating hand tracking \n");

        if (hands[0].is_active)
            left_hand_->UpdateHandTracking(&hands[0]);
        if (hands[1].is_active)
            right_hand_->UpdateHandTracking(&hands[1]);
    }

    // cap.release();
}

const char *const *DeviceProvider::GetInterfaceVersions()
{
    return vr::k_InterfaceVersions;
}

void DeviceProvider::RunFrame()
{
}

// todo: what do we want to do here?
void DeviceProvider::EnterStandby() {}

void DeviceProvider::LeaveStandby() {}

bool DeviceProvider::ShouldBlockStandbyMode()
{
    return false;
}

void DeviceProvider::Cleanup()
{
    DriverLog("Mercury Cleaning up!");
    if (is_active_.exchange(false))
    {
        DriverLog("Shutting down hand tracking...");
        hand_tracking_thread_.join();

        DriverLog("Hand tracking shutdown.");
    }
    // oxr_sdl2_hack_stop(&this->sdl2_hack);
}