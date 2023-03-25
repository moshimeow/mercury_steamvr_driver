// This needs to be first or else ...windows.h? winsock2? includes in u_template_historybuf will cause sadness.
#include "device_provider.h"

#include <iostream>
#include <string>
#include <winsock2.h>
#include <windows.h>

// WinSock2
#pragma comment(lib, "ws2_32.lib")

#include "util/path_utils.h"
#include "driver_log.h"
#include "vive/vive_config.h"
#include "../../monado/src/xrt/tracking/hand/mercury/hg_interface.h"
#include "tracking/t_frame_cv_mat_wrapper.hpp"
#include "os/os_time.h"
#include "oxr_sdl2_hack.h"

#include "tracking_subprocess_protocol.hpp"
#include "math/m_relation_history.h"

#include "util/u_json.hpp"

using namespace xrt::auxiliary::util::json;

namespace xat = xrt::auxiliary::tracking;

void DeviceProvider::Monster300HzThread()
{
    while (is_active_)
    {
        int64_t t = os_monotonic_get_ns();

        t -= (int64_t)ht_delay_;

        left_hand_->UpdateWristPose(t);
        right_hand_->UpdateWristPose(t);

        os_nanosleep(U_TIME_1MS_IN_NS * 1);
    }
}

bool DeviceProvider::StartSubprocess()
{
    // Start the subprocess with the local address and port as arguments
    STARTUPINFO startupInfo;
    PROCESS_INFORMATION processInfo;
    ZeroMemory(&startupInfo, sizeof(startupInfo));
    ZeroMemory(&processInfo, sizeof(processInfo));
    startupInfo.cb = sizeof(startupInfo);

    // maybe this is bad?
    // startupInfo.wShowWindow = true;

    std::string rootpath;
    std::string subprocessPath = {};
    GetDriverRootPath(rootpath);
    GetSubprocessPath(subprocessPath);
    std::string commandLine = subprocessPath + " " + std::to_string(ntohs(localAddr.sin_port)) + " \"" + hmd_config + "\" \"" + rootpath + "\" \"" + rootpath + "\\debug.txt\"";
    std::wstring wideCommandLine(commandLine.begin(), commandLine.end());
    DriverLog("Creating subprocess %s!", commandLine.c_str());
    if (!CreateProcess(NULL, commandLine.data(), NULL, NULL, FALSE, 0, NULL, NULL, &startupInfo, &processInfo))
    {
        DriverLog("Error creating subprocess %s:  %d", commandLine.c_str(), WSAGetLastError());
        closesocket(listenSocket);
        WSACleanup();
        return false;
    }
    return true;
}

bool DeviceProvider::SetupListen()
{
    DriverLog("Server: Listening!\n");

    // Listen for the subprocess to connect
    int iResult = listen(listenSocket, SOMAXCONN);
    if (iResult == SOCKET_ERROR)
    {
        DriverLog("Error listening for connection: %d", WSAGetLastError());
        closesocket(listenSocket);
        WSACleanup();
        return false;
    }
    DriverLog("Server: Accepting connection!\n");

    // Accept the connection
    clientSocket = accept(listenSocket, NULL, NULL);
    if (clientSocket == INVALID_SOCKET)
    {
        DriverLog("Error accepting connection: %d", WSAGetLastError());
        closesocket(listenSocket);
        WSACleanup();
        return false;
    }

    // int timeout = 2000;
    // int result = setsockopt(clientSocket, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout));
    // if (result == SOCKET_ERROR)
    // {
    //     std::cerr << "setsockopt failed: " << WSAGetLastError() << std::endl;
    //     closesocket(clientSocket);
    //     WSACleanup();
    //     return false;
    // }

    return true;
}

vr::EVRInitError DeviceProvider::Init(vr::IVRDriverContext *pDriverContext)
{
    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

    InitDriverLog(vr::VRDriverLog());

    // initialise hand tracking
    // std::string hmd_config;
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
    // sockaddr_in localAddr;
    int localAddrLen = sizeof(localAddr);
    iResult = getsockname(listenSocket, (sockaddr *)&localAddr, &localAddrLen);
    if (iResult == SOCKET_ERROR)
    {
        DriverLog("Error getting socket name: %d", WSAGetLastError());
        closesocket(listenSocket);
        WSACleanup();
        return vr::VRInitError_Driver_Failed;
    }

    if (!StartSubprocess())
    {
        return vr::VRInitError_Driver_Failed;
    }

    // initialise the hands
    left_hand_ = std::make_unique<MercuryHandDevice>(vr::TrackedControllerRole_LeftHand);
    right_hand_ = std::make_unique<MercuryHandDevice>(vr::TrackedControllerRole_RightHand);

    // vr::TrackedDeviceClass_GenericTracker seems to not work

    vr::VRServerDriverHost()->TrackedDeviceAdded(left_hand_->GetSerialNumber().c_str(),
                                                 vr::TrackedDeviceClass_Controller,
                                                 left_hand_.get());
    vr::VRServerDriverHost()->TrackedDeviceAdded(right_hand_->GetSerialNumber().c_str(),
                                                 vr::TrackedDeviceClass_Controller,
                                                 right_hand_.get());

    // #define M_EURO_FILTER_HEAD_TRACKING_FCMIN 30.0
    // #define M_EURO_FILTER_HEAD_TRACKING_FCMIN_D 25.0
    // #define M_EURO_FILTER_HEAD_TRACKING_BETA 0.6

    // m_filter_euro_f32_init(&delay_filter_, 3.14, 1, 0.16);
    // m_filter_euro_f32_init(&delay_filter_, 30*500000, 25*500000, 0.6);
    // m_filter_euro_f32_init(&delay_filter_, 0.00000001, 0.00000001, 0.01);
    // m_filter_euro_f32_init(&delay_filter_, 0.00000001, 0.00000001, 0.0001);
    m_filter_euro_f32_init(&delay_filter_, 0.000000001, 0.000000001, 0.0001);

    is_active_ = true;
    hand_tracking_thread_ = std::thread(&DeviceProvider::HandTrackingThread, this);
    monster_300hz_thread_ = std::thread(&DeviceProvider::Monster300HzThread, this);
    unity_communication_thread_ = std::thread(&DeviceProvider::UnityInputCommunicationThread, this);

    return vr::VRInitError_None;
}

const static enum xrt_space_relation_flags valid_flags_ht = (enum xrt_space_relation_flags)(
    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
    XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT);

void hjs_from_tracking_message(const struct tracking_message_hand &msg, xrt_hand_joint_set &set)
{
    set.is_active = msg.tracked;
    if (!set.is_active)
    {
        for (int i = 0; i < XRT_HAND_JOINT_COUNT; i++)
        {
            set.values.hand_joint_set_default[i].relation.relation_flags = XRT_SPACE_RELATION_BITMASK_NONE;
        }
        return;
    }

    for (int i = 0; i < XRT_HAND_JOINT_COUNT; i++)
    {
        set.values.hand_joint_set_default[i].relation.pose = msg.fingers_relative[i];
        set.values.hand_joint_set_default[i].relation.relation_flags = valid_flags_ht;
    }
}

xrt_space_relation handle_wrist_pose(const struct tracking_message_hand &msg)
{
    xrt_space_relation tmp;

    if (!msg.tracked)
    {
        tmp.relation_flags = XRT_SPACE_RELATION_BITMASK_NONE;
        return tmp;
    }
    tmp.relation_flags = valid_flags_ht;

    tmp.pose = msg.wrist;
    return tmp;
}

xrt_space_relation handle_tip_pose(const struct tracking_message_hand &msg)
{
    xrt_space_relation tmp;

    if (!msg.tracked)
    {
        tmp.relation_flags = XRT_SPACE_RELATION_BITMASK_NONE;
        return tmp;
    }
    tmp.relation_flags = valid_flags_ht;

    tmp.pose = msg.pose_raw;
    return tmp;
}

void DeviceProvider::HandTrackingThread()
{

    if (!SetupListen())
    {
        return;
    }

    DriverLog("HandTrackingThread!");

    while (is_active_)
    {
        // Receive data from the subprocess
        int iResult = 0;
        tracking_message message = {};
        iResult = recv(clientSocket, (char *)&message, TMSIZE, MSG_WAITALL);
        // DriverLog("Received! Result: %d", iResult);
        if (iResult == SOCKET_ERROR)
        {
            int last_error = WSAGetLastError();
            // if (WSAGetLastError() == WSAETIMEDOUT)
            // {
            //     DriverLog("recv timed out! The subprocess was probably killed by you because you're compile-edit-debugging!");
            //     break;
            // }
            if (WSAGetLastError() == WSAECONNRESET && TRY_RESTART)
            {
                os_nanosleep(5000L * U_TIME_1MS_IN_NS);
                if (!StartSubprocess())
                {
                    return;
                }
                if (!SetupListen())
                {
                    return;
                }
            }
            else
            {
                DriverLog("Error receiving data: %d", WSAGetLastError());
                closesocket(clientSocket);
                closesocket(listenSocket);
                WSACleanup();
                return;
            }
        }

        if (iResult != TMSIZE)
        {
            DriverLog("Message was the wrong size: %d", iResult);
            // Sleep a long time so that we don't spam logs
            os_nanosleep(500 * U_TIME_1MS_IN_NS);
            continue;
        }

        // DriverLog(TM_FMT(message));

        // if (message.is_just_keepalive)
        // {
        //     continue;
        // }

        uint64_t now = os_monotonic_get_ns();

        xrt_hand_joint_set hands[2] = {};

        hjs_from_tracking_message(message.hands[0], hands[0]);
        hjs_from_tracking_message(message.hands[1], hands[1]);

        // left_hand_->UpdateFingerPose(&hands[0]);
        // right_hand_->UpdateFingerPose(&hands[1]);

        left_hand_->hand_joint_set_wrist_local = hands[0];
        right_hand_->hand_joint_set_wrist_local = hands[1];

        xrt_space_relation wrists[2] = {};
        xrt_space_relation tips[2] = {};

        wrists[0] = handle_wrist_pose(message.hands[0]);
        wrists[1] = handle_wrist_pose(message.hands[1]);

        tips[0] = handle_tip_pose(message.hands[0]);
        tips[1] = handle_tip_pose(message.hands[1]);

        int64_t max_delay = 0;
        for (int i = 0; i < prev_delays_.size(); i++)
        {
            max_delay = std::max(max_delay, *prev_delays_.get_at_age(i));
        }

        int64_t delay = now - message.camera_timestamp;

        if (message.hands[0].tracked || message.hands[1].tracked)
        {

            prev_delays_.push_back(delay);

            // 1 frametime so that we always have a frame to interpolate extra with
            max_delay += int64_t(float(U_TIME_1S_IN_NS * 1) / (54.0f));

            // [HACK] Extra time because I feel like it.
            // This should be fixed by tuning the euro filter to go *up* faster.
            max_delay += U_TIME_1MS_IN_NS * 10;

            float delay_f = float(max_delay);

            m_filter_euro_f32_run(&delay_filter_, message.camera_timestamp, &delay_f, &ht_delay_);
        }

        m_relation_history_push(left_hand_->wrist_hist_, &wrists[0], message.camera_timestamp);
        m_relation_history_push(right_hand_->wrist_hist_, &wrists[1], message.camera_timestamp);

        m_relation_history_push(left_hand_->pose_raw_hist_, &tips[0], message.camera_timestamp);
        m_relation_history_push(right_hand_->pose_raw_hist_, &tips[1], message.camera_timestamp);

        left_hand_->CursedUpdateCurlAndTrigger(message.hands[0].bs);
        right_hand_->CursedUpdateCurlAndTrigger(message.hands[1].bs);

#ifdef TIMING_DEBUGGING
        timestamps_debug_ << std::to_string(message.camera_timestamp) << ", "
                          << std::to_string(message.host_recieved_frame_timestamp) << ", "
                          << std::to_string(message.sent_at_timestamp) << ", "
                          << std::to_string(now) << ", "
                          << std::to_string(delay) << ", "
                          << std::to_string(max_delay) << ", "
                          << std::to_string(ht_delay_) << std::endl;

        timestamps_debug_.flush();
#endif
    }
}

void blah(const char *stuff, emulated_buttons_state *buttons_lr)
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
        emulated_buttons_state &bs = buttons_lr[i];

        JSONNode side = node[names[i]];
        bs.a = side["a"].asInt();
        bs.b = side["b"].asInt();

        // defaults
        bs.trigger = side["trigger"].asInt();
        bs.grip = side["grip"].asInt();
        bs.system = side["system"].asInt();

        bs.thumbstick_gesture = side["thumbstick_active"].asBool();

        bs.thumbstick_x = side["thumbstick_x"].asDouble();
        bs.thumbstick_y = side["thumbstick_y"].asDouble();

        // printf("%s: %d %d %d %d  %d %f %f\n", names[i].c_str(), a, b, trigger, grip, thumbstick_active, thumbstick_x, thumbstick_y);
    }
}

void DeviceProvider::UnityInputCommunicationThread()
{
    sockaddr_in localAddr;
    SOCKET tClientSocket;
    SOCKET tListenSocket;

    // Initialize Winsock
    WSADATA wsaData;
    int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != 0)
    {
        DriverLog("Unity: WSAStartup failed: %d", iResult);
        return;
    }

    // Create a socket for the subprocess to connect to
    tListenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (tListenSocket == INVALID_SOCKET)
    {
        DriverLog("Unity: Error creating socket: %d", WSAGetLastError());
        WSACleanup();
        return;
    }

    // Bind the socket to any available address and port 0 to let the operating system choose a free port
    sockaddr_in listenAddr;
    listenAddr.sin_family = AF_INET;
    listenAddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    listenAddr.sin_port = htons(12428);
    iResult = bind(tListenSocket, (sockaddr *)&listenAddr, sizeof(listenAddr));
    if (iResult == SOCKET_ERROR)
    {
        DriverLog("Unity: Error binding socket: %d", WSAGetLastError());
        closesocket(tListenSocket);
        WSACleanup();
        return;
    }

    // Get the local address and port of the socket
    // sockaddr_in localAddr;
    int localAddrLen = sizeof(localAddr);
    iResult = getsockname(tListenSocket, (sockaddr *)&localAddr, &localAddrLen);
    if (iResult == SOCKET_ERROR)
    {
        DriverLog("Unity: Error getting socket name: %d", WSAGetLastError());
        closesocket(tListenSocket);
        WSACleanup();
        return;
    }

    // SetupListen

    DriverLog("Unity: Server: Listening!\n");

    // Listen for the subprocess to connect
    iResult = listen(tListenSocket, SOMAXCONN);
    if (iResult == SOCKET_ERROR)
    {
        DriverLog("Unity: Error listening for connection: %d", WSAGetLastError());
        closesocket(tListenSocket);
        WSACleanup();
        return;
    }
    DriverLog("Unity: Server: Accepting connection!\n");

    // Accept the connection
    tClientSocket = accept(tListenSocket, NULL, NULL);
    if (tClientSocket == INVALID_SOCKET)
    {
        DriverLog("Unity: Error accepting connection: %d", WSAGetLastError());
        closesocket(tListenSocket);
        WSACleanup();
        return;
    }

    // end SetupListen

    while (true)
    {

        int iResult = 0;
        char message[2048] = {};
        iResult = recv(tClientSocket, message, 2047, 0);

        // DriverLog("Unity: Result: %d", iResult);

        // if (iResult == 0)
        // {
        //     DriverLog("Unity: Got packet with length zero?");
        //     os_nanosleep(500LL * U_TIME_1MS_IN_NS);
        //     continue;
        // }

        // This isn't really right but should do fine
        if ((iResult == SOCKET_ERROR) && ((WSAGetLastError() == WSAECONNRESET) && TRY_RESTART) || (iResult == 0))
        {
            int last_error = WSAGetLastError();
            // if (WSAGetLastError() == WSAETIMEDOUT)
            // {
            //     DriverLog("recv timed out! The subprocess was probably killed by you because you're compile-edit-debugging!");
            //     break;
            // }
            os_nanosleep(5000L * U_TIME_1MS_IN_NS);

            // close(tClientSocket);



            // SetupListen

            DriverLog("Unity: Server: Restarting and Listening!\n");

            // Listen for the subprocess to connect
            iResult = listen(tListenSocket, SOMAXCONN);
            if (iResult == SOCKET_ERROR)
            {
                DriverLog("Unity: Restarting and Error listening for connection: %d", WSAGetLastError());
                closesocket(tListenSocket);
                WSACleanup();
                return;
            }
            DriverLog("Unity: Restarting and Server: Accepting connection!\n");

            // Accept the connection
            tClientSocket = accept(tListenSocket, NULL, NULL);
            if (tClientSocket == INVALID_SOCKET)
            {
                DriverLog("Unity: Error accepting connection: %d", WSAGetLastError());
                closesocket(tListenSocket);
                WSACleanup();
                return;
            }

            // end SetupListen
        }

        emulated_buttons_state states[2];

        blah(message, states);

        left_hand_->CursedUpdateOtherInputs(states[0]);
        right_hand_->CursedUpdateOtherInputs(states[1]);

        // printf("Got message %s\n\n\n\n", message);
    }
}

const char *const *DeviceProvider::GetInterfaceVersions()
{
    return vr::k_InterfaceVersions;
}

void DeviceProvider::RunFrame()
{
}

// todo: what do we want to do here?
void DeviceProvider::EnterStandby()
{
}

void DeviceProvider::LeaveStandby()
{
}

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
}
