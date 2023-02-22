#include <iostream>
#include <string>
#include <winsock2.h>
#include <windows.h>

#pragma comment(lib, "ws2_32.lib")
#include <winsock2.h>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <fstream>

#include "xrt/xrt_config.h"

#include "vive/vive_config.h"

#include "tracking/t_hand_tracking.h"
#include "tracking/t_frame_cv_mat_wrapper.hpp"
#include "math/m_space.h"
#include "hg_interface.h"

#include "os/os_time.h"
#include "../../src/steamvr_driver/oxr_sdl2_hack.h"
#include "CameraIdxGuesser.hpp"
#include "tracking_subprocess_protocol.hpp"

namespace xat = xrt::auxiliary::tracking;

struct subprocess_state
{
    const char *port;
    const char *vive_config_location;
    bool running = true;

    SOCKET connectSocket;

    cv::VideoCapture cap;

    struct t_hand_tracking_sync *sync = nullptr;
};

std::string read_file(std::string_view path)
{
    constexpr auto read_size = std::size_t(4096);
    auto stream = std::ifstream(path.data());
    stream.exceptions(std::ios_base::badbit);

    std::string out = std::string();
    auto buf = std::string(read_size, '\0');
    while (stream.read(&buf[0], read_size))
    {
        out.append(buf, 0, stream.gcount());
    }
    out.append(buf, 0, stream.gcount());
    return out;
}

bool setup_camera_and_ht(subprocess_state &state)
{
    MessageBoxA(nullptr, "Index index", "Meow meow meow", MB_OK);

    int match_idx = -1;

    for (int i = 0; i < 10; i++)
    {
        match_idx = GetIndexIndex();
        if (match_idx != -1)
        {
            break;
        }
    }
    if (match_idx == -1)
    {
        return false;
    }

    MessageBoxA(nullptr, "Read file", "Meow meow meow", MB_OK);

    std::string config_string = read_file(state.vive_config_location);

    struct vive_config c = {};

    const char *e = config_string.c_str();

    char *j = (char *)malloc(sizeof(char) * 500000);

    strcpy(j, e);

    vive_config_parse(&c, j, U_LOGGING_ERROR);

    struct t_stereo_camera_calibration *calib = NULL;

    xrt_pose head_in_left; // unused

    vive_get_stereo_camera_calibration(&c, &calib, &head_in_left);

    // This definitely needs to be first
    void *sdl2_hack;
    oxr_sdl2_hack_create(&sdl2_hack);

    // zero-initialized out of paranoia
    struct t_camera_extra_info info = {};

    info.views[0].camera_orientation = CAMERA_ORIENTATION_0;
    info.views[1].camera_orientation = CAMERA_ORIENTATION_0;

    info.views[0].boundary_type = HT_IMAGE_BOUNDARY_CIRCLE;
    info.views[1].boundary_type = HT_IMAGE_BOUNDARY_CIRCLE;

    //!@todo This changes by like 50ish pixels from device to device. For now, the solution is simple: just
    //! make the circle a bit bigger than we'd like.
    // Maybe later we can do vignette calibration? Write a tiny optimizer that tries to fit Index's
    // gradient? Unsure.
    info.views[0].boundary.circle.normalized_center.x = 0.5f;
    info.views[0].boundary.circle.normalized_center.y = 0.5f;

    info.views[1].boundary.circle.normalized_center.x = 0.5f;
    info.views[1].boundary.circle.normalized_center.y = 0.5f;

    info.views[0].boundary.circle.normalized_radius = 0.55;
    info.views[1].boundary.circle.normalized_radius = 0.55;

    state.sync =
        t_hand_tracking_sync_mercury_create(calib, info, "C:\\dev\\mercury_steamvr_driver\\src\\steamvr_driver\\mercury\\resources\\internal\\hand-tracking-models\\");

    xrt_frame_context blah = {};

    oxr_sdl2_hack_start(sdl2_hack, NULL, NULL);

    // This is definitely fragile.
    // On Moshi's Windows 10 desktop,  {cv::CAP_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_NONE} is absolutely needed - grabbing the camera is really flaky and eventually you need to restart to get it without those flags.
    // We probably want to steal OpenCV's cap_msmf.cpp at some point to make sure it's solid and won't change on us.

    state.cap = cv::VideoCapture(match_idx, cv::CAP_MSMF, {cv::CAP_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_NONE});

    state.cap.set(cv::CAP_PROP_MODE, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    state.cap.set(cv::CAP_PROP_FPS, 54.0);

    return true;
}

void hjs_to_tracking_message(xrt_hand_joint_set &set, struct tracking_message_hand &msg)
{

    if (!set.is_active)
    {
        msg.tracked = false;
    }
    msg.tracked = true;

    xrt_space_relation wrist = set.values.hand_joint_set_default[XRT_HAND_JOINT_WRIST].relation;

    msg.wrist = wrist.pose;

    for (int i = 0; i < XRT_HAND_JOINT_COUNT; i++)
    {
        struct xrt_relation_chain xrc = {};
        xrt_space_relation tmp = {};
        m_relation_chain_push_relation(&xrc, &set.values.hand_joint_set_default[i].relation);
        m_relation_chain_push_inverted_relation(&xrc, &wrist);
        m_relation_chain_resolve(&xrc, &tmp);

        msg.fingers_relative[i] = tmp.pose;
    }
}

int meow_exit()
{
    std::cout << "Press any key to exit.";
    std::cin.ignore();
    std::cin.get();

    return 1;
}

#define meow_printf printf

int main(int argc, char **argv)
{
    MessageBoxA(nullptr, "Meow", "Meow meow meow", MB_OK);
    if (argc < 3)
    {
        U_LOG_E("Need a port and vive config location");
        meow_exit();
    }
    subprocess_state state = {};

    // MessageBoxA(nullptr, "Meow", "Meow meow meow", MB_OK);

    state.port = argv[1];
    state.vive_config_location = argv[2];

    meow_printf("Port is %s, config location is %s", state.port, state.vive_config_location);

    // Initialize WinSock!
    // This is an "out" struct, which we won't bother with looking at.
    WSADATA wsaData;

    MessageBoxA(nullptr, "WSAStartup", "Meow meow meow", MB_OK);

    // It's also supposedly fine to have multiple overlapping calls to WSAStartup and WSACleanup. Wow Windows commits some *crimes* but alrighty
    int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != 0)
    {
        std::cerr << "WSAStartup failed: " << iResult << std::endl;
        meow_exit();
    }

    MessageBoxA(nullptr, "Socket", "Meow meow meow", MB_OK);

    // Create a socket to connect to the parent process
    state.connectSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (state.connectSocket == INVALID_SOCKET)
    {
        std::cerr << "Error creating socket: " << WSAGetLastError() << std::endl;
        WSACleanup();
        meow_exit();
    }

    MessageBoxA(nullptr, "Connect", "Meow meow meow", MB_OK);

    // Connect to the parent process
    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");

    serverAddr.sin_port = htons(atoi(state.port));
    iResult = connect(state.connectSocket, reinterpret_cast<SOCKADDR *>(&serverAddr), sizeof(serverAddr));
    if (iResult == SOCKET_ERROR)
    {
        std::cerr << "Error connecting to server: " << WSAGetLastError() << std::endl;
        closesocket(state.connectSocket);
        WSACleanup();
        meow_exit();
    }
    MessageBoxA(nullptr, "Camera", "Meow meow meow", MB_OK);

    setup_camera_and_ht(state);

    MessageBoxA(nullptr, "Add vars", "Meow meow meow", MB_OK);

    u_var_add_root(&state, "SteamVR driver!", 0);

    u_var_add_bool(&state, &state.running, "Running");

    while (state.running)
    {
        cv::Mat mat_ = {};
        cv::Mat mat = {};

        // Often returns [ WARN:0@11.016] global C:\dev\vcpkg\buildtrees\opencv4\src\4.6.0-9a95a1b699.clean\modules\videoio\src\cap_msmf.cpp (1752) CvCapture_MSMF::grabFrame videoio(MSMF): can't grab frame. Error: -2147483638
        // Grrrr.
        bool success = state.cap.read(mat_);

        if (!success)
        {
            U_LOG_E("Failed!");
            break;
        }

        //!@todo
        double time_now = os_monotonic_get_ns();
        double time_camera = state.cap.get(cv::CAP_PROP_POS_MSEC) * 1e6;

        double time_ratio = time_now / time_camera;

        double time_diff = time_camera - time_now;

        // I've seen it vary from -4 to -30ms, really interesting!

        double time_diff_ms = (double)time_diff / (double)U_TIME_1MS_IN_NS;

        cv::cvtColor(mat_, mat, cv::COLOR_BGR2GRAY);

        cv::Mat mats_grayscale[2];

        // xat::FrameMat fms[2]; // = {};
        xrt_frame *frames[2] = {NULL, NULL};

        mats_grayscale[0] = mat(cv::Rect(0, 0, 960, 960));
        mats_grayscale[1] = mat(cv::Rect(960, 0, 960, 960));

        for (int i = 0; i < 2; i++)
        {
            xat::FrameMat::Params params;
            params.stereo_format = XRT_STEREO_FORMAT_NONE;
            params.timestamp_ns = time_camera;
            xat::FrameMat::wrapL8(mats_grayscale[i], &frames[i], params);
        }

        struct xrt_hand_joint_set hands[2];
        uint64_t out_timestamp;
        t_ht_sync_process(state.sync, frames[0], frames[1], &hands[0], &hands[1], &out_timestamp);

        tracking_message message = {};

        message.size = TMSIZE;
        message.timestamp = time_camera;
        hjs_to_tracking_message(hands[0], message.hands[0]);
        hjs_to_tracking_message(hands[1], message.hands[1]);

        // Send data to the parent process
        // char *sendBuffer = ;
        std::cout << "Going to send!" << std::endl;
        iResult = send(state.connectSocket, (const char *)&message, TMSIZE, 0);
        if (iResult == SOCKET_ERROR)
        {
            std::cerr << "Error sending data: " << WSAGetLastError() << std::endl;
            break;
        }
    }

    state.cap.release();

    // return 0;

    // // Receive data from the parent process
    // char recvBuffer[1024];
    // iResult = recv(state.connectSocket, recvBuffer, sizeof(recvBuffer), 0);
    // if (iResult == SOCKET_ERROR)
    // {
    //     std::cerr << "Error receiving data: " << WSAGetLastError() << std::endl;
    //     closesocket(state.connectSocket);
    //     WSACleanup();
    //     return 1;
    // }
    // recvBuffer[iResult] = '\0';
    // std::cout << "Received data from parent process: " << recvBuffer << std::endl;

    // Close the socket and cleanup Winsock
    closesocket(state.connectSocket);
    WSACleanup();

    return 0;
}