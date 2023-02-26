#pragma once

#include <atomic>
#include <string>

#include "openvr_driver.h"
#include "tracking/t_hand_tracking.h"
#include "xrt/xrt_defines.h"
#include "math/m_space.h"
#include "util/bones.h"
#include "tracking_subprocess_protocol.hpp"

// From OpenGloves
// enum KnuckleDeviceComponentIndex
// {
//     kSystemClick = 0,
//     kSystemTouch,
//     kTriggerClick,
//     kTriggerValue,
//     kTrackpadX,
//     kTrackpadY,
//     //   kTrackpadTouch,
//     //   kTrackpadForce,
//     kGripTouch,
//     kGripForce,
//     kGripValue,
//     //   kThumbstickClick,
//     kThumbstickTouch,
//     kThumbstickX,
//     kThumbstickY,
//     kAClick,
//     kATouch,
//     kBClick,
//     kBTouch,
//     kFingerIndex,
//     kFingerMiddle,
//     kFingerRing,
//     kFingerPinky,

//     //   kSkeleton,
//     kCount
// };

enum KnuckleDeviceComponentIndex {
  kKnuckleDeviceComponentIndex_SystemClick = 0,
  kKnuckleDeviceComponentIndex_SystemTouch,
  kKnuckleDeviceComponentIndex_TriggerClick,
  kKnuckleDeviceComponentIndex_TriggerValue,
//   kKnuckleDeviceComponentIndex_TrackpadX,
//   kKnuckleDeviceComponentIndex_TrackpadY,
//   kKnuckleDeviceComponentIndex_TrackpadTouch,
//   kKnuckleDeviceComponentIndex_TrackpadForce,
  kKnuckleDeviceComponentIndex_GripTouch,
  kKnuckleDeviceComponentIndex_GripForce,
  kKnuckleDeviceComponentIndex_GripValue,
  kKnuckleDeviceComponentIndex_ThumbstickClick,
  kKnuckleDeviceComponentIndex_ThumbstickTouch,
  kKnuckleDeviceComponentIndex_ThumbstickX,
  kKnuckleDeviceComponentIndex_ThumbstickY,
  kKnuckleDeviceComponentIndex_AClick,
  kKnuckleDeviceComponentIndex_ATouch,
  kKnuckleDeviceComponentIndex_BClick,
  kKnuckleDeviceComponentIndex_BTouch,
  kKnuckleDeviceComponentIndex_FingerIndex,
  kKnuckleDeviceComponentIndex_FingerMiddle,
  kKnuckleDeviceComponentIndex_FingerRing,
  kKnuckleDeviceComponentIndex_FingerPinky,

  kKnuckleDeviceComponentIndex_Skeleton,
  kKnuckleDeviceComponentIndex_Count
};

class MercuryHandDevice : public vr::ITrackedDeviceServerDriver
{
public:
    explicit MercuryHandDevice(vr::ETrackedControllerRole role);
    ~MercuryHandDevice();

    vr::EVRInitError Activate(uint32_t unObjectId) override;

    void UpdateFingerPose(const xrt_hand_joint_set *joint_set_local, xrt_pose raw, xrt_pose wrist);

    void Deactivate() override;

    void EnterStandby() override;

    void *GetComponent(const char *pchComponentNameAndVersion) override;

    void DebugRequest(const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize) override;

    void UpdateWristPose(uint64_t timestamp);

    // void UpdateBooleanAndScalar(bool &value, bool &last_value, )

    void UpdateFakeControllerInput(struct emulated_buttons_state buttons_state);

    vr::DriverPose_t GetPose() override;
    std::string GetSerialNumber();

    struct m_relation_history *wrist_hist_;
    struct m_relation_history *pose_raw_hist_;

    xrt_hand_joint_set hand_joint_set_wrist_local;
    vr::VRBoneTransform_t bone_transforms_[OPENVR_BONE_COUNT];
    vr::ETrackedControllerRole role_;

    vr::VRInputComponentHandle_t input_components_[kKnuckleDeviceComponentIndex_Count];
    // vr::VRInputComponentHandle_t trigger_click_;
    // vr::VRInputComponentHandle_t trigger_value_;

    // bool hand_is_active_ = false;

    emulated_buttons_state buttons_state_;

    // bool trigger_ = false;

private:
    bool IsLeftHand();

    vr::TrackedDeviceIndex_t device_id_ = -1;

    vr::VRInputComponentHandle_t skeleton_component_handle_;

    std::atomic<bool> has_activated_;

    int64_t last_time_printed_predicted_message_ = 0;
};