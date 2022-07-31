
#pragma once

// #include "openvr_driver.h"
#include "stdafx.h"

class CLeapPoller;
class CLeapControllerIndex;
class CLeapStation;

class CServerDriver final : public vr::IServerTrackedDeviceProvider
{
    enum LeapControllerHand : size_t
    {
        LCH_Left = 0U,
        LCH_Right = 1U,

        LCH_Count
    };

    static const char* const ms_interfaces[];

    bool m_connectionState;
    CLeapPoller *m_leapPoller;
    CLeapControllerIndex *m_controllers[LCH_Count];
    CLeapStation *m_leapStation;

    CServerDriver(const CServerDriver &that) = delete;
    CServerDriver& operator=(const CServerDriver &that) = delete;

    void TryToPause();

    // vr::IServerTrackedDeviceProvider
    vr::EVRInitError Init(vr::IVRDriverContext *pDriverContext);
    void Cleanup();
    const char* const* GetInterfaceVersions();
    void RunFrame();
    bool ShouldBlockStandbyMode();
    void EnterStandby();
    void LeaveStandby();
public:
    // Todo what should the constructor do?
    // CServerDriver();
    // ~CServerDriver();

    void ProcessExternalMessage(const char *p_message);
};
