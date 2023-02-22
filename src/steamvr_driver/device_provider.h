#pragma once

#include <memory>
#include <atomic>
#include <thread>
#include <winsock2.h>

#include "openvr_driver.h"

#include "mercury_device.h"

class DeviceProvider : public vr::IServerTrackedDeviceProvider {
public:
    vr::EVRInitError Init(vr::IVRDriverContext *pDriverContext) override;

    void Cleanup() override;

    const char *const *GetInterfaceVersions() override;

    void RunFrame() override;

    bool ShouldBlockStandbyMode() override;

    void EnterStandby() override;

    void LeaveStandby() override;

private:

    void HandTrackingThread();

    SOCKET clientSocket;
    SOCKET listenSocket;

    std::atomic<bool> is_active_;
    std::thread hand_tracking_thread_;

    std::unique_ptr<MercuryHandDevice> left_hand_;
    std::unique_ptr<MercuryHandDevice> right_hand_;

    // videoInput video_input_;
    int camera_idx;

    
    void* sdl2_hack = NULL;
};