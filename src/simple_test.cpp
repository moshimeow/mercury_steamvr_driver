// Copyright 2022, Collabora, Inc.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The thing is here!
 * @author Moses Turner <moses@collabora.com>
 */


#include <opencv2/opencv.hpp>

#include <fstream>
#include <string>

// #include "monado/src/xrt/include/xrt/xrt_config.h"
#include "xrt/xrt_config.h"

#include "vive/vive_config.h"

#include "tracking/t_hand_tracking.h"
#include "tracking/t_frame_cv_mat_wrapper.hpp"
#include "../monado/src/xrt/tracking/hand/mercury/hg_interface.h"

#include "os/os_time.h"
#include "steamvr_driver/oxr_sdl2_hack.h"
#include "CameraIdxGuesser.hpp"

namespace xat = xrt::auxiliary::tracking;


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

int main()
{

    int match_idx = GetIndexIndex();
    if (match_idx == -1) {
        return -1;
    }

    std::string t20_config = read_file("C:\\dev\\mercury_steamvr_driver\\attic\\T20_config.json");

    std::cout << t20_config << std::endl;

    struct vive_config c = {};

    const char *e = t20_config.c_str();

    char *j = (char *)malloc(sizeof(char) * 500000);

    strcpy(j, e);

    vive_config_parse(&c, j, U_LOGGING_DEBUG);

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

    struct t_hand_tracking_sync *sync =
        t_hand_tracking_sync_mercury_create(calib, info, "C:\\dev\\mercury_steamvr_driver\\hand-tracking-models\\");

    xrt_frame_context blah = {};

    t_hand_tracking_async *async = t_hand_tracking_async_default_create(&blah, sync);

    oxr_sdl2_hack_start(sdl2_hack, NULL, NULL);

    // This is definitely fragile.
    // On Moshi's Windows 10 desktop,  {cv::CAP_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_NONE} is absolutely needed - grabbing the camera is really flaky and eventually you need to restart to get it without those flags.
    // We probably want to steal OpenCV's cap_msmf.cpp at some point to make sure it's solid and won't change on us.

    cv::VideoCapture cap(match_idx, cv::CAP_MSMF, {cv::CAP_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_NONE});

    cap.set(cv::CAP_PROP_MODE, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    cap.set(cv::CAP_PROP_FPS, 54.0);

    while (true)
    {
        cv::Mat mat_ = {};
        cv::Mat mat = {};

        // Often returns [ WARN:0@11.016] global C:\dev\vcpkg\buildtrees\opencv4\src\4.6.0-9a95a1b699.clean\modules\videoio\src\cap_msmf.cpp (1752) CvCapture_MSMF::grabFrame videoio(MSMF): can't grab frame. Error: -2147483638
        // Grrrr.
        bool success = cap.read(mat_);

        if (!success)
        {
            U_LOG_E("Failed!");
            break;
        }

        //!@todo
        double time_now = os_monotonic_get_ns();
        double time_camera = cap.get(cv::CAP_PROP_POS_MSEC) * 1e6;

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

        xrt_sink_push_frame(async->sinks.left, frames[0]);
        xrt_sink_push_frame(async->sinks.right, frames[1]);

        U_LOG_E("meow DIFF %f %f %f %f", time_diff_ms, time_now, time_camera, time_ratio);
    }

    cap.release();

    return 0;
}
