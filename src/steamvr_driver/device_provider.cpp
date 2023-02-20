#include "device_provider.h"
#include "util/path_utils.h"
#include "driver_log.h"
#include "vive/vive_config.h"
#include "../../monado/src/xrt/tracking/hand/mercury/hg_interface.h"
#include "tracking/t_frame_cv_mat_wrapper.hpp"
#include "os/os_time.h"
#include "oxr_sdl2_hack.h"
#include "CameraIdxGuesser.hpp"

namespace xat = xrt::auxiliary::tracking;

// // I'm cargo-culting this. I have no idea why we're not just using a header. Probably fix soon.
// /* ---- HACK ---- */
// extern int
// oxr_sdl2_hack_create(void **out_hack);

// // Why are we using xinst or xsysd? wtf is this shit
// extern void
// oxr_sdl2_hack_start(void *hack, struct xrt_instance *xinst, struct xrt_system_devices *xsysd);

// extern void
// oxr_sdl2_hack_stop(void **hack_ptr);
// /* ---- HACK ---- */

vr::EVRInitError DeviceProvider::Init(vr::IVRDriverContext *pDriverContext)
{
    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

    InitDriverLog(vr::VRDriverLog());

    int match_idx = GetIndexIndex();
    if (match_idx == -1) {
        return vr::VRInitError_Driver_CalibrationInvalid;
    }
    this->camera_idx = match_idx;

    // initialise hand tracking
    std::string hmd_config;
    if (!GetHMDConfig(hmd_config))
    {
        DriverLog("Failed to find HMD config. Exiting.");
        return vr::VRInitError_Driver_CalibrationInvalid;
    }

    struct vive_config config = {};
    char *j = (char *)malloc(sizeof(char) * 500000);
    strcpy(j, hmd_config.c_str());

    vive_config_parse(&config, j, U_LOGGING_DEBUG);

    struct t_stereo_camera_calibration *calib = NULL;
    xrt_pose head_in_left;
    vive_get_stereo_camera_calibration(&config, &calib, &head_in_left);

    // oxr_sdl2_hack_create(&this->sdl2_hack);

    //!@todo This changes by like 50ish pixels from device to device. For now, the solution is simple: just
    //! make the circle a bit bigger than we'd like.
    // Maybe later we can do vignette calibration? Write a tiny optimizer that tries to fit Index's
    // gradient? Unsure.
    struct t_camera_extra_info info;
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

    info.views[0].boundary.circle.normalized_radius = 0.5;
    info.views[1].boundary.circle.normalized_radius = 0.5;

    info.views[0].camera_orientation = CAMERA_ORIENTATION_0;
    info.views[1].camera_orientation = CAMERA_ORIENTATION_0;

    std::string htmodels_path = {};

    GetHandTrackingModelsPath(htmodels_path);
    DriverLog("Output path is %s", htmodels_path.c_str());


    struct t_hand_tracking_sync *sync =
        t_hand_tracking_sync_mercury_create(calib, info, htmodels_path.c_str());

    // oxr_sdl2_hack_start(this->sdl2_hack, NULL, NULL);


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
    hand_tracking_thread_ = std::thread(&DeviceProvider::HandTrackingThread, this, sync, match_idx);

    return vr::VRInitError_None;
}

void DeviceProvider::HandTrackingThread(t_hand_tracking_sync *sync, int camera_id)
{
    cv::VideoCapture cap(camera_id, cv::CAP_MSMF, {cv::CAP_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_NONE});

    cap.set(cv::CAP_PROP_MODE, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    cap.set(cv::CAP_PROP_FPS, 54.0);

    while (is_active_)
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
        struct xrt_hand_joint_set hands[2];
        uint64_t out_timestamp;
        t_ht_sync_process(sync, frames[0], frames[1], &hands[0], &hands[1], &out_timestamp);

         if (hands[0].is_active)
            left_hand_->UpdateHandTracking(&hands[0]);
        if (hands[1].is_active)
            right_hand_->UpdateHandTracking(&hands[1]);

        for (int i = 0; i < 2; i++)
        {
            // Don't need 'em
            xrt_frame_reference(&frames[i], NULL);
        }


        // DriverLog("meow DIFF %f %f %f %f", time_diff_ms, time_now, time_camera, time_ratio);
    }

    cap.release();
    
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