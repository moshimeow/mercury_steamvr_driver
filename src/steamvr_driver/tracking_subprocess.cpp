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
#include "openvr.h"
#include "u_subprocess_logging.h"

#include "pinch_decider.hpp"
#include "math/m_filter_one_euro.h"
#include "everything_else_decider.hpp"

#define meow_printf U_SP_LOG_E

// #define FCMIN 30.0
// #define FCMIN_D 25.0
// #define OUR_BETA 0.6

#define FCMIN 10.0
#define FCMIN_D 9.0
#define OUR_BETA 3.0
#define OUR_BETA_PINCHED 0.5

// #define FCMIN_QUAT 30.0*20000.5
// #define FCMIN_D_QUAT 25.0*20000.5

// #define FCMIN_QUAT 0.6
// #define FCMIN_D_QUAT 0.55
// #define OUR_BETA_QUAT 0.000006

// #define FCMIN_QUAT 0.6
// #define FCMIN_D_QUAT 0.55
// #define OUR_BETA_QUAT 0.000006

#define FCMIN_QUAT 1.6
#define FCMIN_D_QUAT 1.5
#define OUR_BETA_QUAT 3.0
#define OUR_BETA_PINCHED_QUAT 0.2

namespace xat = xrt::auxiliary::tracking;

struct subprocess_state
{
    const char *port;
    const char *vive_config_location;
    bool running = true;

    SOCKET connectSocket;

    cv::VideoCapture cap;

    vr::IVRSystem *vr_system;

    xrt_pose left_camera_in_head;

    struct t_hand_tracking_sync *sync = nullptr;

    // Left wrist, left indpxm, right wrist, right indpxm, head.
    // struct m_filter_euro_vec3 filters[3];
    struct m_filter_euro_vec3 vector_filters[2];
    struct m_filter_euro_quat quat_filters[2];

    struct emulated_buttons_state bs[2] = {};
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

// void reinit_filters(subprocess_state &state)

bool setup_camera_and_ht(subprocess_state &state)
{
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

    meow_printf("Read file");

    std::string config_string = read_file(state.vive_config_location);

    struct vive_config c = {};

    const char *e = config_string.c_str();

    char *j = (char *)malloc(sizeof(char) * 500000);

    strcpy(j, e);

    vive_config_parse(&c, j, U_LOGGING_ERROR);

    struct t_stereo_camera_calibration *calib = NULL;

    xrt_pose head_in_left; // unused

    vive_get_stereo_camera_calibration(&c, &calib, &head_in_left);

    math_pose_invert(&head_in_left, &state.left_camera_in_head);

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

    // // Suggestions. These are suitable for head tracking.

    for (int i = 0; i < 2; i++)
    {
        m_filter_euro_vec3_init(&state.vector_filters[i], FCMIN, FCMIN_D, OUR_BETA);
        m_filter_euro_quat_init(&state.quat_filters[i], FCMIN_QUAT, FCMIN_D_QUAT, OUR_BETA_QUAT);
    }
    return true;
}

xrt_pose GetPose(const vr::HmdMatrix34_t &matrix)
{
    xrt_quat q{};
    xrt_vec3 v = {matrix.m[0][3], matrix.m[1][3], matrix.m[2][3]};

    q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;

    q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
    q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
    q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);

    return {q, v};
}

void hjs2_to_tracking_message(subprocess_state &state, xrt_hand_joint_set sets[2], xrt_pose attached_head, struct tracking_message &msg, int64_t tracking_ts)
{
    bool tracked[2] = {false, false};
    xrt_pose wrist_global[2] = {};
    xrt_pose index_pxm_global[2] = {};

    for (int hand_idx = 0; hand_idx < 2; hand_idx++)
    {
        xrt_hand_joint_set &set = sets[hand_idx];

        msg.hands[hand_idx].tracked = set.is_active;
        tracked[hand_idx] = set.is_active;
        if (!msg.hands[hand_idx].tracked)
        {
            state.vector_filters[hand_idx].base.have_prev_y = false;
            state.quat_filters[hand_idx].base.have_prev_y = false;
            // m_filter_euro_vec3_init(&state.vector_filters[hand_idx], FCMIN, FCMIN_D, OUR_BETA);
            // m_filter_euro_quat_init(&state.quat_filters[hand_idx], FCMIN_QUAT, FCMIN_D_QUAT, OUR_BETA_QUAT);
            continue;
        }

        trigger_decide(set, &state.bs[hand_idx].trigger);
        msg.hands[hand_idx].bs = state.bs[hand_idx];

        xrt_space_relation wrist = set.values.hand_joint_set_default[XRT_HAND_JOINT_WRIST].relation;
        xrt_space_relation index_pxm = set.values.hand_joint_set_default[XRT_HAND_JOINT_INDEX_PROXIMAL].relation;

        {
            struct xrt_relation_chain xrc = {};
            xrt_space_relation tmp = {};
            m_relation_chain_push_relation(&xrc, &wrist);
            // TODO ADD HEAD OFFSET HERE
            m_relation_chain_push_pose(&xrc, &state.left_camera_in_head);
            m_relation_chain_push_pose(&xrc, &attached_head);

            m_relation_chain_resolve(&xrc, &tmp);

            wrist_global[hand_idx] = tmp.pose;
        }

        {
            struct xrt_relation_chain xrc = {};
            xrt_space_relation tmp = {};
            m_relation_chain_push_relation(&xrc, &index_pxm);
            // TODO ADD HEAD OFFSET HERE
            m_relation_chain_push_pose(&xrc, &state.left_camera_in_head);
            m_relation_chain_push_pose(&xrc, &attached_head);

            m_relation_chain_resolve(&xrc, &tmp);

            index_pxm_global[hand_idx] = tmp.pose;
        }
    }

    for (int hand_idx = 0; hand_idx < 2; hand_idx++)
    {
        if (!tracked[hand_idx])
        {
            continue;
        }
        xrt_hand_joint_set &set = sets[hand_idx];

        xrt_pose ap_ = aim_pose(hand_idx, wrist_global[hand_idx], index_pxm_global[hand_idx], tracked[!hand_idx] ? &wrist_global[!hand_idx] : NULL, attached_head);

        xrt_pose ap;

        if (msg.hands[hand_idx].bs.trigger)
        {
            // Too low
            // const float mul = 0.000001;

            // Also too low
            // const float mul = 0.0001;

            // Also too low - these all feel the same and result in ~no movement
            // const float mul = 0.001;

            // This does result in some movement
            // const float mul = 0.01;

            // OK this is good and feels purposeful and generally helps interactions
            const float mul = 0.05;

            state.quat_filters[hand_idx].base.fc_min = FCMIN_QUAT * mul;
            state.quat_filters[hand_idx].base.fc_min_d = FCMIN_D_QUAT * mul;
            state.quat_filters[hand_idx].base.beta = OUR_BETA_PINCHED_QUAT;

            state.vector_filters[hand_idx].base.fc_min = FCMIN * mul;
            state.vector_filters[hand_idx].base.fc_min_d = FCMIN_D * mul;
            state.quat_filters[hand_idx].base.beta = OUR_BETA_PINCHED;
        }
        else
        {
            state.quat_filters[hand_idx].base.fc_min = FCMIN_QUAT;
            state.quat_filters[hand_idx].base.fc_min_d = FCMIN_D_QUAT;

            state.vector_filters[hand_idx].base.fc_min = FCMIN;
            state.vector_filters[hand_idx].base.fc_min_d = FCMIN_D;
        }

        m_filter_euro_quat_run(&state.quat_filters[hand_idx], tracking_ts, &ap_.orientation, &ap.orientation);
        m_filter_euro_vec3_run(&state.vector_filters[hand_idx], tracking_ts, &ap_.position, &ap.position);

        msg.hands[hand_idx].pose_raw = ap;
        msg.hands[hand_idx].wrist = wrist_global[hand_idx];
        for (int i = 0; i < XRT_HAND_JOINT_COUNT; i++)
        {
            struct xrt_relation_chain xrc = {};
            xrt_space_relation tmp = {};
            m_relation_chain_push_relation(&xrc, &set.values.hand_joint_set_default[i].relation);
            m_relation_chain_push_inverted_pose_if_not_identity(&xrc, &wrist_global[hand_idx]);
            m_relation_chain_resolve(&xrc, &tmp);

            msg.hands[hand_idx].fingers_relative[i] = tmp.pose;
        }
    }
    decide_everything_else(msg, attached_head);
    state.bs[0] = msg.hands[0].bs;
    state.bs[1] = msg.hands[1].bs;
}

bool check_vrserver_alive(subprocess_state &state)
{
    vr::VREvent_t event{};
    while (state.vr_system->PollNextEvent(&event, sizeof(event)))
    {
        meow_printf("Got event %d", event.eventType);
        switch (event.eventType)
        {
        case vr::VREvent_Quit:
        {
            meow_printf("VRServer quitting!");
            return false;
        }
        break;
        default:
            return true;
        }
    }
    return true;
}

int meow_exit()
{
    meow_printf("Press any key to exit.");
    std::cin.ignore();
    std::cin.get();

    abort();
}

int main(int argc, char **argv)
{
    meow_printf("Starting!");

    if (argc < 3)
    {
        U_SP_LOG_E("Need a port and vive config location");
        meow_exit();
    }
    subprocess_state state = {};

    // MessageBoxA(nullptr, "Meow", "Meow meow meow", MB_OK);

    state.port = argv[1];
    state.vive_config_location = argv[2];

    vr::EVRInitError error;

    for (int i = 0; i < 100; i++)
    {

        meow_printf("vriNIT! %s %s", state.port, state.vive_config_location);

        state.vr_system = vr::VR_Init(&error, vr::EVRApplicationType::VRApplication_Background);

        if (error == vr::VRInitError_None)
        {
            break;
        }

        meow_printf("rESULT WAS %d\n", error);

        os_nanosleep(U_TIME_1MS_IN_NS * 100);
    }

    meow_printf("Port is %s, config location is %s", state.port, state.vive_config_location);

    // Initialize WinSock!
    // This is an "out" struct, which we won't bother with looking at.
    WSADATA wsaData;

    meow_printf("WSAStartup");
    // It's also supposedly fine to have multiple overlapping calls to WSAStartup and WSACleanup. Wow Windows commits some *crimes* but alrighty
    int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != 0)
    {
        meow_printf("WSAStartup failed: ", iResult);
        meow_exit();
    }

    // MessageBoxA(nullptr, "Socket", "Meow meow meow", MB_OK);
    meow_printf("Socket");

    // Create a socket to connect to the parent process
    state.connectSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (state.connectSocket == INVALID_SOCKET)
    {
        meow_printf("Error creating socket: ", WSAGetLastError());
        WSACleanup();
        meow_exit();
    }

    meow_printf("Connect");

    // Connect to the parent process
    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");

    serverAddr.sin_port = htons(atoi(state.port));
    iResult = connect(state.connectSocket, reinterpret_cast<SOCKADDR *>(&serverAddr), sizeof(serverAddr));
    if (iResult == SOCKET_ERROR)
    {
        meow_printf("Error connecting to server: ", WSAGetLastError());
        closesocket(state.connectSocket);
        WSACleanup();
        meow_exit();
    }

    meow_printf("Camera");

    setup_camera_and_ht(state);

    meow_printf("Add vars");

    u_var_add_root(&state, "SteamVR driver!", 0);

    u_var_add_bool(&state, &state.running, "Running");

    u_var_add_bool(&state, &state.bs[0].a, "left.a");
    u_var_add_bool(&state, &state.bs[0].b, "left.b");
    u_var_add_bool(&state, &state.bs[0].trigger, "left.trigger");
    u_var_add_bool(&state, &state.bs[0].thumbstick_gesture, "left.thumbstick_gesture");
    u_var_add_f32(&state, &state.bs[0].thumbstick_x, "left.thumbstick_x");
    u_var_add_f32(&state, &state.bs[0].thumbstick_y, "left.thumbstick_y");

    u_var_add_f32(&state, &state.bs[0].curls[0], "left.curls[0]");
    u_var_add_f32(&state, &state.bs[0].curls[1], "left.curls[1]");
    u_var_add_f32(&state, &state.bs[0].curls[2], "left.curls[2]");
    u_var_add_f32(&state, &state.bs[0].curls[3], "left.curls[3]");
    u_var_add_f32(&state, &state.bs[0].curls[4], "left.curls[4]");

    u_var_add_bool(&state, &state.bs[1].a, "right.a");
    u_var_add_bool(&state, &state.bs[1].b, "right.b");
    u_var_add_bool(&state, &state.bs[1].trigger, "right.trigger");
    u_var_add_bool(&state, &state.bs[1].thumbstick_gesture, "right.thumbstick_gesture");
    u_var_add_f32(&state, &state.bs[1].thumbstick_x, "right.thumbstick_x");
    u_var_add_f32(&state, &state.bs[1].thumbstick_y, "right.thumbstick_y");

    u_var_add_f32(&state, &state.bs[1].curls[0], "right.curls[0]");
    u_var_add_f32(&state, &state.bs[1].curls[1], "right.curls[1]");
    u_var_add_f32(&state, &state.bs[1].curls[2], "right.curls[2]");
    u_var_add_f32(&state, &state.bs[1].curls[3], "right.curls[3]");
    u_var_add_f32(&state, &state.bs[1].curls[4], "right.curls[4]");

    while (state.running)
    {
        if (!check_vrserver_alive(state))
        {
            break;
        }
        cv::Mat mat_ = {};
        cv::Mat mat = {};

        // Often returns [ WARN:0@11.016] global C:\dev\vcpkg\buildtrees\opencv4\src\4.6.0-9a95a1b699.clean\modules\videoio\src\cap_msmf.cpp (1752) CvCapture_MSMF::grabFrame videoio(MSMF): can't grab frame. Error: -2147483638
        // Grrrr.
        bool success = state.cap.read(mat_);

        if (!success)
        {
            U_SP_LOG_E("Failed!");
            break;
        }

        //!@todo
        uint64_t time_now_uint = os_monotonic_get_ns();
        double time_now = (double)time_now_uint;
        double time_camera = state.cap.get(cv::CAP_PROP_POS_MSEC) * 1e6;

        double time_ratio = time_now / time_camera;

        double time_diff = time_camera - time_now;

        // I've seen it vary from -4 to -30ms, really interesting!

        double time_diff_s = (double)time_diff / (double)U_TIME_1S_IN_NS;

        vr::TrackedDevicePose_t hmd_pose;

        state.vr_system->GetDeviceToAbsoluteTrackingPose(vr::ETrackingUniverseOrigin::TrackingUniverseRawAndUncalibrated, time_diff_s, &hmd_pose, 1);

        xrt_pose attached_hmd_pose = GetPose(hmd_pose.mDeviceToAbsoluteTracking);

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

        for (int i = 0; i < 2; i++)
        {
            // Don't need 'em
            xrt_frame_reference(&frames[i], NULL);
        }

        tracking_message message = {};

        message.size = TMSIZE;
        message.camera_timestamp = time_camera;
        message.host_recieved_frame_timestamp = time_now_uint;

        hjs2_to_tracking_message(state, hands, attached_hmd_pose, message, time_camera);
        // hjs_to_tracking_message(state, 0, hands[0], attached_hmd_pose, message.hands[0]);
        // hjs_to_tracking_message(state, 1, hands[1], attached_hmd_pose, message.hands[1]);

        // Send data to the parent process
        // char *sendBuffer = ;
        meow_printf("Going to send!");

        message.sent_at_timestamp = os_monotonic_get_ns();

        iResult = send(state.connectSocket, (const char *)&message, TMSIZE, 0);

        if (iResult == SOCKET_ERROR)
        {
            // WSAECONNABORTED = 10053 winerror.h
            meow_printf("Error sending data: %d", WSAGetLastError());
            break;
        }
    }

    meow_printf("Shutting down semi-cleanly!");

    state.cap.release();

    vr::VR_Shutdown();

    // Close the socket and cleanup Winsock
    closesocket(state.connectSocket);
    WSACleanup();

    meow_exit();

    return 0;
}