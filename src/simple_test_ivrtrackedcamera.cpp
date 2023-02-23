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
#include "openvr.h"

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
    vr::IVRSystem *vr_system;
    vr::EVRInitError error;

    vr_system = vr::VR_Init(&error, vr::EVRApplicationType::VRApplication_Background);

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

    vr::EVRTrackedCameraError err;

    vr::IVRTrackedCamera *camera = vr::VRTrackedCamera();

    vr::TrackedCameraHandle_t camera_handle;

    err = camera->AcquireVideoStreamingService(0, &camera_handle);

    printf("AcquireVideoStreamingService error is %s\n", camera->GetCameraErrorNameFromEnum(err));

    int i = 0;
    while (true)
    {

        cv::Mat mat_rgba(cv::Size(1920, 960), CV_8UC4);

        vr::CameraVideoStreamFrameHeader_t header;

#if 0 
        // vr::EVRTrackedCameraError err = camera->GetVideoStreamFrameBuffer(camera_handle, vr::VRTrackedCameraFrameType_Distorted, mat_rgba.data, 1920*960*4, &header, 1);
#elif 1
        vr::EVRTrackedCameraError err = camera->GetVideoStreamFrameBuffer(camera_handle,
                                                                          vr::VRTrackedCameraFrameType_Distorted, mat_rgba.data, 1920 * 960 * 4, &header, sizeof(vr::CameraVideoStreamFrameHeader_t));
#else
        vr::EVRTrackedCameraError err = camera->GetVideoStreamFrameBuffer(camera_handle,
                                                                          vr::VRTrackedCameraFrameType_Distorted, mat_rgba.data, 1920 * 960 * 4, &header, i);
#endif

    // VRTrackedCameraError_InvalidFrameHeaderVersion
        printf("error is %s\n", camera->GetCameraErrorNameFromEnum(err));

        cv::imshow("meow", mat_rgba);
        if (cv::waitKey(1) == 'q') {
            break;
        }
        i++;
    }

    err = camera->ReleaseVideoStreamingService(camera_handle);

    exit(0);
    return 0;
}
