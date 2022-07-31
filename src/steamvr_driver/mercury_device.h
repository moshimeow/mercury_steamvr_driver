#pragma once

#include <atomic>
#include <string>

#include "openvr_driver.h"
#include "tracking/t_hand_tracking.h"
#include "util/bones.h"


class MercuryHandDevice : public vr::ITrackedDeviceServerDriver {
public:

    explicit MercuryHandDevice(vr::ETrackedControllerRole role);

    vr::EVRInitError Activate(uint32_t unObjectId) override;

    void UpdateHandTracking(const xrt_hand_joint_set *joint_set);

    void Deactivate() override;

    void EnterStandby() override;

    void *GetComponent(const char *pchComponentNameAndVersion) override;

    void DebugRequest(const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize) override;

    vr::DriverPose_t GetPose() override;
    std::string GetSerialNumber();

private:
    bool IsLeftHand();

    vr::TrackedDeviceIndex_t device_id_;
    vr::ETrackedControllerRole role_;

    vr::VRBoneTransform_t bone_transforms_[OPENVR_BONE_COUNT];
    vr::VRInputComponentHandle_t skeleton_component_handle_;

    std::atomic<bool> has_activated_;

    int shadow_controller_ = -1;
};