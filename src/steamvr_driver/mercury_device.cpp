#include "mercury_device.h"

#include "driver_log.h"
#include "math/m_relation_history.h"
#include "os/os_time.h"

MercuryHandDevice::MercuryHandDevice(vr::ETrackedControllerRole role) : role_(role)
{
    m_relation_history_create(&this->wrist_hist_);
    m_relation_history_create(&this->pose_raw_hist_);
}

MercuryHandDevice::~MercuryHandDevice()
{
    m_relation_history_destroy(&this->wrist_hist_);
    m_relation_history_destroy(&this->pose_raw_hist_);
}

vr::EVRInitError MercuryHandDevice::Activate(uint32_t unObjectId)
{
    device_id_ = unObjectId;

    vr::PropertyContainerHandle_t props = vr::VRProperties()->TrackedDeviceToPropertyContainer(
        device_id_);
    vr::VRProperties()->SetInt32Property(props, vr::Prop_ControllerHandSelectionPriority_Int32, 2147483647);
    vr::VRProperties()->SetStringProperty(props, vr::Prop_SerialNumber_String, GetSerialNumber().c_str());
    vr::VRProperties()->SetBoolProperty(props, vr::Prop_DeviceIsWireless_Bool, true);
    vr::VRProperties()->SetBoolProperty(props, vr::Prop_DeviceIsCharging_Bool, false);
    vr::VRProperties()->SetBoolProperty(props, vr::Prop_DeviceCanPowerOff_Bool, false);
    vr::VRProperties()->SetBoolProperty(props, vr::Prop_Identifiable_Bool, false);
    vr::VRProperties()->SetStringProperty(props, vr::Prop_ResourceRoot_String, "mercury");

    // these registered device type strings used to point to index controllers,
    // I changed them to mercury hands, but either way, including them causes the hand tracking
    // to break.
    /*vr::VRProperties()->SetStringProperty(props, vr::Prop_RegisteredDeviceType_String,
                                           IsLeftHand() ? "collabora/mercury_hand_l"
                                                        : "collabora/mercury_hand_r");*/
    vr::VRProperties()->SetStringProperty(props, vr::Prop_ResourceRoot_String, "mercury");
    vr::VRProperties()->SetStringProperty(props, vr::Prop_RenderModelName_String, !IsLeftHand() ? "{mercury}mercury_right" : "{mercury}mercury_left");
    vr::VRProperties()->SetStringProperty(props, vr::Prop_ControllerType_String, "knuckles");
    vr::VRProperties()->SetStringProperty(props, vr::Prop_ManufacturerName_String, "Valve");
    vr::VRProperties()->SetStringProperty(props, vr::Prop_TrackingSystemName_String, "yeet");

    // we need to supply a controller profile so that bindings can work.
    // this path might be wrong. I checked the documentation though, and the docs say it is
    // using the right form.
    vr::VRProperties()->SetStringProperty(props, vr::Prop_InputProfilePath_String, "{mercury}/resources/input/index_controller_profile.json");
    // vr::VRProperties()->SetStringProperty(props, vr::Prop_RenderModelName_String, !IsLeftHand() ? "{mercury}valve_controller_knu_1_0_right" : "{mercury}valve_controller_knu_1_0_left");

    vr::VRDriverInput()->CreateSkeletonComponent(
        props,
        IsLeftHand() ? "/input/skeleton/left" : "/input/skeleton/right",
        IsLeftHand() ? "/skeleton/hand/left" : "/skeleton/hand/right",
        "/pose/raw",
        vr::EVRSkeletalTrackingLevel::VRSkeletalTracking_Full,
        bone_transforms_,
        OPENVR_BONE_COUNT,
        &skeleton_component_handle_);

    {
        vr::VRDriverInput()->CreateBooleanComponent(props, "/input/system/click", &input_components_[kKnuckleDeviceComponentIndex_SystemClick]);
        vr::VRDriverInput()->CreateBooleanComponent(props, "/input/system/touch", &input_components_[kKnuckleDeviceComponentIndex_SystemTouch]);

        vr::VRDriverInput()->CreateBooleanComponent(props, "/input/trigger/click", &input_components_[kKnuckleDeviceComponentIndex_TriggerClick]);
        vr::VRDriverInput()->CreateBooleanComponent(props, "/input/trigger/value", &input_components_[kKnuckleDeviceComponentIndex_TriggerValue]);

        // vr::VRDriverInput()->CreateScalarComponent(props, "/input/trackpad/x", &input_components_[kKnuckleDeviceComponentIndex_TrackpadX], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
        // vr::VRDriverInput()->CreateScalarComponent(props, "/input/trackpad/y", &input_components_[kKnuckleDeviceComponentIndex_TrackpadY], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);

        vr::VRDriverInput()->CreateBooleanComponent(props, "/input/grip/touch", &input_components_[kKnuckleDeviceComponentIndex_GripTouch]);
        vr::VRDriverInput()->CreateScalarComponent(props, "/input/grip/force", &input_components_[kKnuckleDeviceComponentIndex_GripForce], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
        vr::VRDriverInput()->CreateScalarComponent(props, "/input/grip/value", &input_components_[kKnuckleDeviceComponentIndex_GripValue], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);

        vr::VRDriverInput()->CreateBooleanComponent(props, "/input/thumbstick/click", &input_components_[kKnuckleDeviceComponentIndex_ThumbstickClick]);
        vr::VRDriverInput()->CreateBooleanComponent(props, "/input/thumbstick/touch", &input_components_[kKnuckleDeviceComponentIndex_ThumbstickTouch]);
        vr::VRDriverInput()->CreateScalarComponent(props, "/input/thumbstick/x", &input_components_[kKnuckleDeviceComponentIndex_ThumbstickX], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
        vr::VRDriverInput()->CreateScalarComponent(props, "/input/thumbstick/y", &input_components_[kKnuckleDeviceComponentIndex_ThumbstickY], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);

        vr::VRDriverInput()->CreateBooleanComponent(props, "/input/a/click", &input_components_[kKnuckleDeviceComponentIndex_AClick]);
        vr::VRDriverInput()->CreateBooleanComponent(props, "/input/a/touch", &input_components_[kKnuckleDeviceComponentIndex_ATouch]);
        vr::VRDriverInput()->CreateBooleanComponent(props, "/input/b/click", &input_components_[kKnuckleDeviceComponentIndex_BClick]);
        vr::VRDriverInput()->CreateBooleanComponent(props, "/input/b/touch", &input_components_[kKnuckleDeviceComponentIndex_BTouch]);

        vr::VRDriverInput()->CreateScalarComponent(props, "/input/finger/index", &input_components_[kKnuckleDeviceComponentIndex_FingerIndex], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
        vr::VRDriverInput()->CreateScalarComponent(props, "/input/finger/middle", &input_components_[kKnuckleDeviceComponentIndex_FingerMiddle], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
        vr::VRDriverInput()->CreateScalarComponent(props, "/input/finger/ring", &input_components_[kKnuckleDeviceComponentIndex_FingerRing], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
        vr::VRDriverInput()->CreateScalarComponent(props, "/input/finger/pinky", &input_components_[kKnuckleDeviceComponentIndex_FingerPinky], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
    }
    // below, as-is, broke hand tracking input. Unsure.
    // The role hint is different from the role, so this should be fine.
    // vr::VRProperties()->SetInt32Property(props, vr::Prop_ControllerRoleHint_Int32, IsLeftHand() ? vr::TrackedControllerRole_RightHand : vr::TrackedControllerRole_LeftHand);

    // What does this do?
    // vr::VRProperties()->SetUint64Property(props, vr::Prop_SupportedButtons_Uint64, 0);

    this->has_activated_ = true;

    return vr::VRInitError_None;
}

vr::HmdVector3d_t GetPosition(const vr::HmdMatrix34_t &matrix)
{
    return {matrix.m[0][3], matrix.m[1][3], matrix.m[2][3]};
}

vr::HmdQuaternion_t GetRotation(const vr::HmdMatrix34_t &matrix)
{
    vr::HmdQuaternion_t q{};

    q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;

    q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
    q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
    q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);

    return q;
}

template <class T, class U>
void convert_quaternion(const T &p_quatA, U &p_quatB)
{
    p_quatB.x = p_quatA.x;
    p_quatB.y = p_quatA.y;
    p_quatB.z = p_quatA.z;
    p_quatB.w = p_quatA.w;
}

template <class T, class U>
void convert_vec3(const T &p_quatA, U &p_quatB)
{
    p_quatB.x = p_quatA.x;
    p_quatB.y = p_quatA.y;
    p_quatB.z = p_quatA.z;
}

// Updates the poses of the fake controllers as well as the finger poses
void MercuryHandDevice::UpdateFingerPose(const xrt_hand_joint_set *joint_set_local, xrt_pose raw, xrt_pose wrist)
{
    if (!this->has_activated_)
    {
        return;
    }

    if (!joint_set_local->is_active)
    {
        return;
    }

    struct xrt_relation_chain xrc = {};
    struct xrt_space_relation tmp = {};

    m_relation_chain_push_pose(&xrc, &wrist);
    m_relation_chain_push_inverted_pose_if_not_identity(&xrc, &raw);
    m_relation_chain_resolve(&xrc, &tmp);

    // Gets the finger poses relative to... the "root"
    // It's constantly up for debate what "the root" actually is
    HandJointSetToBoneTransform(*joint_set_local, this->bone_transforms_, role_, tmp.pose);

    // Sets the finger poses
    //! @todo: do we really need to update it twice?
    vr::VRDriverInput()->UpdateSkeletonComponent(skeleton_component_handle_, vr::VRSkeletalMotionRange_WithController,
                                                 bone_transforms_, OPENVR_BONE_COUNT);
    vr::VRDriverInput()->UpdateSkeletonComponent(skeleton_component_handle_,
                                                 vr::VRSkeletalMotionRange_WithoutController, bone_transforms_,
                                                 OPENVR_BONE_COUNT);
}

void MercuryHandDevice::UpdateWristPose(uint64_t timestamp)
{
    if (!this->has_activated_)
    {
        return;
    }

    vr::DriverPose_t pose{};
    pose.qDriverFromHeadRotation.w = 1;
    pose.qWorldFromDriverRotation.w = 1;

    pose.deviceIsConnected = true;
    pose.poseTimeOffset = 0.0;

    //! @todo Fragile! Depends on hand_joint_set_default->hand_relation being identity.
    struct xrt_space_relation raw_pose_in_global;
    struct xrt_space_relation wrist_pose_in_global;

    enum m_relation_history_result result = m_relation_history_get(pose_raw_hist_, timestamp, &raw_pose_in_global);
    m_relation_history_get(wrist_hist_, timestamp, &wrist_pose_in_global); // enum m_relation_history_result result =

    if (result == M_RELATION_HISTORY_RESULT_PREDICTED)
    {
        static int64_t now = os_monotonic_get_ns();
        if (now - last_time_printed_predicted_message_ > U_TIME_1MS_IN_NS * 50)
        {
            DriverLog("Predicted this pose! Shouldn't happen!");
        }

        last_time_printed_predicted_message_ = now;

        // pose.result = vr::TrackingResult_Running_OutOfRange;
        // pose.poseIsValid = false;
        // goto update;
    }

    if (result == M_RELATION_HISTORY_RESULT_INVALID)
    {
        pose.result = vr::TrackingResult_Running_OutOfRange;
        pose.poseIsValid = false;
        goto update;
    }

    if (raw_pose_in_global.relation_flags == XRT_SPACE_RELATION_BITMASK_NONE)
    {
        pose.result = vr::TrackingResult_Running_OutOfRange;
        pose.poseIsValid = false;
        goto update;
    }

    pose.result = vr::TrackingResult_Running_OK;
    pose.poseIsValid = true;

    for (int i = 0; i < 3; i++)
    {
        pose.vecWorldFromDriverTranslation[i] = 0.0;
        pose.vecAngularVelocity[i] = 0.0f;
        pose.vecVelocity[i] = 0.0f;
    }

    pose.qWorldFromDriverRotation = {1, 0, 0, 0};

    pose.vecPosition[0] = raw_pose_in_global.pose.position.x;
    pose.vecPosition[1] = raw_pose_in_global.pose.position.y;
    pose.vecPosition[2] = raw_pose_in_global.pose.position.z;

    convert_quaternion(raw_pose_in_global.pose.orientation, pose.qRotation);

// Update the fake controller pose
update:
    UpdateFingerPose(&this->hand_joint_set_wrist_local, raw_pose_in_global.pose, wrist_pose_in_global.pose);
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(device_id_, pose, sizeof(vr::DriverPose_t));
}

void curl(MercuryHandDevice &dev, enum KnuckleDeviceComponentIndex idx, float curl_val)
{
    vr::VRDriverInput()->UpdateScalarComponent(dev.input_components_[idx], curl_val, 0);
}

// Note: It doesn't matter if we constantly send updates with the same value, that's fine. This function strives _not_ to just because it's a waste, but it still does for some things. It's fine.
void MercuryHandDevice::UpdateFakeControllerInput(emulated_buttons_state buttons_state)
{

    if (buttons_state_.system != buttons_state.system)
    {
        // DriverLog("Updated ! %d", buttons_state.system);
        vr::VRDriverInput()->UpdateBooleanComponent(input_components_[kKnuckleDeviceComponentIndex_SystemClick], buttons_state.system, 0);
        vr::VRDriverInput()->UpdateBooleanComponent(input_components_[kKnuckleDeviceComponentIndex_SystemTouch], buttons_state.system, 0);
    }

    if (buttons_state_.trigger != buttons_state.trigger)
    {
        // DriverLog("Updated ! %d", buttons_state.trigger);
        vr::VRDriverInput()->UpdateBooleanComponent(input_components_[kKnuckleDeviceComponentIndex_TriggerClick], buttons_state.trigger, 0);
        vr::VRDriverInput()->UpdateScalarComponent(input_components_[kKnuckleDeviceComponentIndex_TriggerValue], buttons_state.trigger ? 1.0f : 0.0f, 0);
    }

    if (buttons_state_.a != buttons_state.a)
    {
        bool val = buttons_state.a;
        vr::VRDriverInput()->UpdateBooleanComponent(input_components_[kKnuckleDeviceComponentIndex_AClick], val, 0);
        vr::VRDriverInput()->UpdateBooleanComponent(input_components_[kKnuckleDeviceComponentIndex_ATouch], val, 0);
    }

    if (buttons_state_.b != buttons_state.b)
    {
        bool val = buttons_state.b;
        vr::VRDriverInput()->UpdateBooleanComponent(input_components_[kKnuckleDeviceComponentIndex_BClick], val, 0);
        vr::VRDriverInput()->UpdateBooleanComponent(input_components_[kKnuckleDeviceComponentIndex_BTouch], val, 0);
    }

    if (buttons_state_.grip != buttons_state.grip)
    {
        bool val = buttons_state.grip;
        float val_scalar = val ? 1.0f : 0.0f;
        vr::VRDriverInput()->UpdateBooleanComponent(input_components_[kKnuckleDeviceComponentIndex_GripTouch], val, 0);
        vr::VRDriverInput()->UpdateScalarComponent(input_components_[kKnuckleDeviceComponentIndex_GripForce], val_scalar, 0);
        vr::VRDriverInput()->UpdateScalarComponent(input_components_[kKnuckleDeviceComponentIndex_GripValue], val_scalar, 0);
    }

    if (buttons_state_.thumbstick_gesture)
    {
        vr::VRDriverInput()->UpdateScalarComponent(input_components_[kKnuckleDeviceComponentIndex_ThumbstickX], buttons_state_.thumbstick_x, 0);
        vr::VRDriverInput()->UpdateScalarComponent(input_components_[kKnuckleDeviceComponentIndex_ThumbstickY], buttons_state_.thumbstick_y, 0);

        vr::VRDriverInput()->UpdateBooleanComponent(input_components_[kKnuckleDeviceComponentIndex_ThumbstickTouch], true, 0);
    }
    else
    {

        vr::VRDriverInput()->UpdateScalarComponent(input_components_[kKnuckleDeviceComponentIndex_ThumbstickX], 0, 0);
        vr::VRDriverInput()->UpdateScalarComponent(input_components_[kKnuckleDeviceComponentIndex_ThumbstickY], 0, 0);
        vr::VRDriverInput()->UpdateBooleanComponent(input_components_[kKnuckleDeviceComponentIndex_ThumbstickTouch], false, 0);
    }

    vr::VRDriverInput()->UpdateScalarComponent(input_components_[kKnuckleDeviceComponentIndex_FingerIndex], buttons_state_.curls[1] * -0.4, 0);
    vr::VRDriverInput()->UpdateScalarComponent(input_components_[kKnuckleDeviceComponentIndex_FingerMiddle], buttons_state_.curls[2] * -0.4, 0);
    vr::VRDriverInput()->UpdateScalarComponent(input_components_[kKnuckleDeviceComponentIndex_FingerRing], buttons_state_.curls[3] * -0.4, 0);
    vr::VRDriverInput()->UpdateScalarComponent(input_components_[kKnuckleDeviceComponentIndex_FingerPinky], buttons_state_.curls[4] * -0.4, 0);

    buttons_state_ = buttons_state;
}

// todo: what do we want to do here?
void MercuryHandDevice::EnterStandby() {}

void *MercuryHandDevice::GetComponent(const char *pchComponentNameAndVersion)
{
    return nullptr;
}

// used when an application wants to debug a driver. maybe useful, probably not.
void MercuryHandDevice::DebugRequest(const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize)
{
    if (unResponseBufferSize >= 1)
        pchResponseBuffer[0] = 0;
}

// this is basically never called. just return an empty pose.
vr::DriverPose_t MercuryHandDevice::GetPose()
{
    DriverLog("GetPose called for role: %i. Returning empty pose", role_);

    vr::DriverPose_t pose{};
    return pose;
}

void MercuryHandDevice::Deactivate()
{
}

std::string MercuryHandDevice::GetSerialNumber()
{
    std::string base = "MERCURY1-";

    return base + (IsLeftHand() ? "LEFT" : "RIGHT");
}

bool MercuryHandDevice::IsLeftHand()
{
    return role_ == vr::TrackedControllerRole_LeftHand;
}