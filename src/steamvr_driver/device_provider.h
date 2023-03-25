#pragma once

#include "math/m_filter_one_euro.h"
#include "util/u_template_historybuf.hpp"
#include "util/u_time.h"

#include "mercury_device.h"
#include "openvr_driver.h"

#include <iostream>
#include <fstream>
#include <memory>
#include <atomic>
#include <thread>
#include <winsock2.h>

#undef TIMING_DEBUGGING
#define TRY_RESTART 1

#define SUBPROCESS_STOP_START

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

    bool StartSubprocess();
    bool SetupListen();

    void HandTrackingThread();
    void UnityInputCommunicationThread();

    SOCKET clientSocket;
    SOCKET listenSocket;

    // SOCKET restarterListenSocket;
    // SOCKET restarterClientSocket;

    std::atomic<bool> is_active_;
    std::thread hand_tracking_thread_;
    std::thread monster_300hz_thread_;
    std::thread unity_communication_thread_;

    std::unique_ptr<MercuryHandDevice> left_hand_;
    std::unique_ptr<MercuryHandDevice> right_hand_;

    float ht_delay_ = U_TIME_1MS_IN_NS * 55;

    m_filter_euro_f32 delay_filter_;

    xrt::auxiliary::util::HistoryBuffer<int64_t, 25> prev_delays_;

    sockaddr_in localAddr;
    std::string hmd_config;


    // struct m_relation_history *relation_hist[2];

#ifdef TIMING_DEBUGGING
    std::ofstream timestamps_debug_ = std::ofstream("C:\\dev\\timestamps.csv");
#endif
};