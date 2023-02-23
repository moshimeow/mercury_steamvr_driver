#pragma once

#include <atomic>
#include <string>

#include "openvr_driver.h"
#include "tracking/t_hand_tracking.h"
#include "xrt/xrt_defines.h"
#include "math/m_space.h"
#include "util/bones.h"

class MercuryHandDevice : public vr::ITrackedDeviceServerDriver
{
public:
    explicit MercuryHandDevice(vr::ETrackedControllerRole role);
    ~MercuryHandDevice();

    vr::EVRInitError Activate(uint32_t unObjectId) override;

    void UpdateFingerPose(const xrt_hand_joint_set *joint_set);

    void Deactivate() override;

    void EnterStandby() override;

    void *GetComponent(const char *pchComponentNameAndVersion) override;

    void DebugRequest(const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize) override;

    void UpdateWristPose(uint64_t timestamp, xrt_pose head_pose);

    vr::DriverPose_t GetPose() override;
    std::string GetSerialNumber();


    struct m_relation_history *wrist_hist_;
    xrt_hand_joint_set hand_joint_set_wrist_local;
    vr::VRBoneTransform_t bone_transforms_[OPENVR_BONE_COUNT];
    vr::ETrackedControllerRole role_;

    // bool hand_is_active_ = false;

private:
    bool IsLeftHand();

    vr::TrackedDeviceIndex_t device_id_;

    vr::VRInputComponentHandle_t skeleton_component_handle_;

    std::atomic<bool> has_activated_;


};