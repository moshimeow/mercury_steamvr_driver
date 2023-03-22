// Copyright 2023, Moshi Turner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The thing is here!
 * @author Moshi Turner <mosesturner@protonmail.com>
 */

#include <opencv2/opencv.hpp>

#include <fstream>
#include <string>

#include "openvr.h"
#include "os/os_time.h"

int main()
{
    vr::IVRSystem *vr_system;
    vr::EVRInitError error;

    vr_system = vr::VR_Init(&error, vr::EVRApplicationType::VRApplication_Overlay);

    vr::EVRTrackedCameraError err;

    vr::IVRTrackedCamera *camera = vr::VRTrackedCamera();

    vr::TrackedCameraHandle_t camera_handle;

    err = camera->AcquireVideoStreamingService(0, &camera_handle);

    printf("AcquireVideoStreamingService error is %s\n", camera->GetCameraErrorNameFromEnum(err));
    cv::Mat mat_rgba(cv::Size(1920, 960), CV_8UC4);

    uint32_t last_sequence = 0;

    int i = 0;
    while (true)
    {

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

        printf("error is %s, i is %d, last_sequence is %u, this_sequence is %u \n", camera->GetCameraErrorNameFromEnum(err), i, last_sequence, header.nFrameSequence);

        last_sequence = header.nFrameSequence;

        cv::imshow("meow", mat_rgba);
        if (cv::waitKey(16) == 'q') {
            break;
        }
        // Hypothesis: SteamVR doesn't like us pulling more than one frame per frame.
        // If we sleep for a really long time, will it work?
        os_nanosleep(U_TIME_1MS_IN_NS*500);
        i++;
    }

    err = camera->ReleaseVideoStreamingService(camera_handle);

    exit(0);
    return 0;
}
