#pragma once

#include <memory>
#include <atomic>
#include <thread>
#include <winsock2.h>

#include "openvr_driver.h"

#include "mercury_device.h"
#include "math/m_filter_one_euro.h"
#include "util/u_time.h"
#include <iostream>
#include <fstream>

class DeviceProvider : public vr::IServerTrackedDeviceProvider {
public:
    vr::EVRInitError Init(vr::IVRDriverContext *pDriverContext) override;

    void Cleanup() override;

    const char *const *GetInterfaceVersions() override;

    void RunFrame() override;

    bool ShouldBlockStandbyMode() override;

    void EnterStandby() override;

    void LeaveStandby() override;

    void Monster300HzThread();

private:

    void HandTrackingThread();

    SOCKET clientSocket;
    SOCKET listenSocket;

    std::atomic<bool> is_active_;
    std::thread hand_tracking_thread_;
    std::thread monster_300hz_thread_;

    std::unique_ptr<MercuryHandDevice> left_hand_;
    std::unique_ptr<MercuryHandDevice> right_hand_;

    uint64_t ht_delay_ = U_TIME_1MS_IN_NS * 100;

    m_filter_euro_f32 *filter;

    // struct m_relation_history *relation_hist[2];


    std::ofstream timestamps_debug_ = std::ofstream("C:\\dev\\timestamps.csv");
    
    void* sdl2_hack = NULL;
};