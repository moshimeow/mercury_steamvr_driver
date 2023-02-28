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

    vr_system = vr::VR_Init(&error, vr::EVRApplicationType::VRApplication_Background);

    vr::EVRTrackedCameraError err;

    vr::IVRTrackedCamera *camera = vr::VRTrackedCamera();

    vr::TrackedCameraHandle_t camera_handle;

    err = camera->AcquireVideoStreamingService(0, &camera_handle);

    printf("AcquireVideoStreamingService error is %s\n", camera->GetCameraErrorNameFromEnum(err));

    int i = 0;
    uint32_t last_sequence = 0;
    int64_t last_time_asked = 0;
    while (true)
    {
        int64_t now = os_monotonic_get_ns();
        int64_t extra_time = (now - last_time_asked);

        printf("extra %f ", (double)extra_time / U_TIME_1MS_IN_NS);

        if (extra_time < (U_TIME_1MS_IN_NS * 18))
        {

            printf("Sleeping %f mseconds", (double)extra_time / U_TIME_1MS_IN_NS);
            os_nanosleep(extra_time);

            continue;
        }
        last_time_asked = now;

        cv::Mat mat_rgba(cv::Size(1920, 960), CV_8UC4);

        vr::CameraVideoStreamFrameHeader_t header = {};

        vr::EVRTrackedCameraError err = camera->GetVideoStreamFrameBuffer(camera_handle,
                                                                          vr::VRTrackedCameraFrameType_Distorted, NULL, 0, &header, sizeof(vr::CameraVideoStreamFrameHeader_t));

        printf("error (get sequence) is %s\n", camera->GetCameraErrorNameFromEnum(err));

        if (err != vr::VRTrackedCameraError_None)
        {
            os_nanosleep(U_TIME_1MS_IN_NS * 10);
            continue;
        }
        if (last_sequence == header.nFrameSequence)
        {
            printf("Continuing because sequence was the same (%u %u) \n", last_sequence, header.nFrameSequence);
            os_nanosleep(U_TIME_1MS_IN_NS * 10);
            continue;
        }
        last_sequence = header.nFrameSequence;

        printf("sequence is %u\n", last_sequence);

        err = camera->GetVideoStreamFrameBuffer(camera_handle,
                                                vr::VRTrackedCameraFrameType_Distorted, mat_rgba.data, 1920 * 960 * 4, &header, sizeof(vr::CameraVideoStreamFrameHeader_t));

        // VRTrackedCameraError_InvalidFrameHeaderVersion
        printf("error (get frame) is %s\n", camera->GetCameraErrorNameFromEnum(err));

        cv::imshow("meow", mat_rgba);

        os_nanosleep(U_TIME_1MS_IN_NS * 10);
        if (cv::waitKey(1) == 'q')
        {
            break;
        }
        i++;
    }

    err = camera->ReleaseVideoStreamingService(camera_handle);

    exit(0);
    return 0;
}
