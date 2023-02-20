#include "device_provider.h"
#include "util/vr_utils.h"
#include "util/driver_log.h"
#include "vive/vive_config.h"
#include "../../monado/src/xrt/tracking/hand/mercury/hg_interface.h"
#include "tracking/t_frame_cv_mat_wrapper.hpp"
#include "os/os_time.h"
#include "oxr_sdl2_hack.h"

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

    struct t_hand_tracking_sync *sync =
        t_hand_tracking_sync_mercury_create(calib, info, "C:\\dev\\mercury_steamvr_driver\\hand-tracking-models\\");

    // oxr_sdl2_hack_start(this->sdl2_hack, NULL, NULL);

    video_input_.setVerbose(true);
    int num_devices = video_input_.listDevices();
    DriverLog("Devices found: %i", num_devices);

    std::vector<std::string> devs = video_input_.getDeviceList();

    int wanted_idx = 0;
    bool is_camera = false;
    for (int i = 0; i < devs.size(); i++)
    {
        std::cout << devs[i] << std::endl;
        is_camera = devs[i] == "eTronVideo";
        std::cout << is_camera << std::endl;
        if (is_camera)
        {
            wanted_idx = i;
            is_camera = true;
            break;
        }
    }
    if (!is_camera)
    {
        DriverLog("Unable to find camera on device!");

        // ¯\_(ツ)_/¯
        return vr::VRInitError_Driver_CalibrationInvalid;
    }

    // This seems to correctly limit it to 54Hz
    video_input_.setUseCallback(true);
    video_input_.setIdealFramerate(wanted_idx, 54);
    video_input_.setupDevice(wanted_idx, 1920, 960);

    video_input_.setRequestedMediaSubType(6);

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
    hand_tracking_thread_ = std::thread(&DeviceProvider::HandTrackingThread, this, sync, wanted_idx);

    return vr::VRInitError_None;
}

void DeviceProvider::HandTrackingThread(t_hand_tracking_sync *sync, int camera_id)
{
    int width = video_input_.getWidth(camera_id);
    int height = video_input_.getHeight(camera_id);
    int size = video_input_.getSize(camera_id);
    DriverLog("Height: %i, Width: %i, Size: %i", height, width, size);

    unsigned char *buf = new unsigned char[size];
    while (is_active_)
    {
        video_input_.getPixels(camera_id, buf, false, true);
        // cv::Mat(full_size, CV_8UC3, debug_frame->data, debug_frame->stride);

        uint64_t time = os_monotonic_get_ns();

        cv::Mat eh = cv::Mat(cv::Size(1920, 960), CV_8UC3, buf, 1920 * 3);

        cv::Mat mats_rgb[2];
        cv::Mat mats_grayscale[2];

        // xat::FrameMat fms[2]; // = {};
        xrt_frame *frames[2] = {NULL, NULL};

        mats_rgb[0] = eh(cv::Rect(0, 0, 960, 960));
        mats_rgb[1] = eh(cv::Rect(960, 0, 960, 960));

        for (int i = 0; i < 2; i++)
        {
            cv::cvtColor(mats_rgb[i], mats_grayscale[i], cv::COLOR_BGR2GRAY);
            xat::FrameMat::Params params;
            params.stereo_format = XRT_STEREO_FORMAT_NONE;
            params.timestamp_ns = time;
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

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    delete[] buf;
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