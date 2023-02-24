#include "mercury_device.h"

#include "driver_log.h"
#include "math/m_relation_history.h"
#include "os/os_time.h"

MercuryHandDevice::MercuryHandDevice(vr::ETrackedControllerRole role) : role_(role)
{
    m_relation_history_create(&this->wrist_hist_);
}

MercuryHandDevice::~MercuryHandDevice()
{
    m_relation_history_destroy(&this->wrist_hist_);
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
    vr::VRProperties()->SetStringProperty(props, vr::Prop_ResourceRoot_String, "indexcontroller");
    vr::VRProperties()->SetStringProperty(props, vr::Prop_RenderModelName_String, !IsLeftHand() ? "{indexcontroller}valve_controller_knu_1_0_right" : "{indexcontroller}valve_controller_knu_1_0_left");
    vr::VRProperties()->SetStringProperty(props, vr::Prop_ControllerType_String, "knuckles");
    vr::VRProperties()->SetStringProperty(props, vr::Prop_ManufacturerName_String, "Valve");
    vr::VRProperties()->SetStringProperty(props, vr::Prop_TrackingSystemName_String, "yeet");

    // we need to supply a controller profile so that bindings can work.
    // this path might be wrong. I checked the documentation though, and the docs say it is
    // using the right form.
    vr::VRProperties()->SetStringProperty(props, vr::Prop_InputProfilePath_String, "{indexcontroller}/resources/input/index_controller_profile.json");
    vr::VRProperties()->SetStringProperty(props, vr::Prop_RenderModelName_String, !IsLeftHand() ? "{indexcontroller}valve_controller_knu_1_0_right" : "{indexcontroller}valve_controller_knu_1_0_left");

    vr::VRDriverInput()->CreateSkeletonComponent(
        props,
        IsLeftHand() ? "/input/skeleton/left" : "/input/skeleton/right",
        IsLeftHand() ? "/skeleton/hand/left" : "/skeleton/hand/right",
        "/pose/raw",
        vr::EVRSkeletalTrackingLevel::VRSkeletalTracking_Full,
        bone_transforms_,
        OPENVR_BONE_COUNT,
        &skeleton_component_handle_);

    // below, as-is, broke hand tracking input. Unsure.
    // The role hint is different from the role, so this should be fine.
    // vr::VRProperties()->SetInt32Property(props, vr::Prop_ControllerRoleHint_Int32, IsLeftHand() ? vr::TrackedControllerRole_RightHand : vr::TrackedControllerRole_LeftHand);
    vr::VRProperties()->SetUint64Property(props, vr::Prop_SupportedButtons_Uint64, 0);

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
void MercuryHandDevice::UpdateFingerPose(const xrt_hand_joint_set *joint_set)
{
    if (!this->has_activated_)
    {
        return;
    }

    if (!joint_set->is_active)
    {
        return;
    }

    // Gets the finger poses relative to... the "root"
    // It's constantly up for debate what "the root" actually is
    HandJointSetToBoneTransform(*joint_set, this->bone_transforms_, role_);

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
    vr::DriverPose_t pose{};
    pose.qDriverFromHeadRotation.w = 1;
    pose.qWorldFromDriverRotation.w = 1;

    pose.deviceIsConnected = true;
    pose.poseTimeOffset = 0.0;

    //! @todo Fragile! Depends on hand_joint_set_default->hand_relation being identity.
    struct xrt_space_relation wrist_pose_in_global;

    enum m_relation_history_result result = m_relation_history_get(wrist_hist_, timestamp, &wrist_pose_in_global);

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

    if (wrist_pose_in_global.relation_flags == XRT_SPACE_RELATION_BITMASK_NONE)
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

    pose.vecPosition[0] = wrist_pose_in_global.pose.position.x;
    pose.vecPosition[1] = wrist_pose_in_global.pose.position.y;
    pose.vecPosition[2] = wrist_pose_in_global.pose.position.z;

    // pose.vecAcceleration[0] = wrist_pose_in_global.linear_velocity.x;
    // pose.vecAcceleration[1] = wrist_pose_in_global.linear_velocity.y;
    // pose.vecAcceleration[2] = wrist_pose_in_global.linear_velocity.z;

    // pose.vecAngularVelocity[0] = wrist_pose_in_global.angular_velocity.x;
    // pose.vecAngularVelocity[1] = wrist_pose_in_global.angular_velocity.y;
    // pose.vecAngularVelocity[2] = wrist_pose_in_global.angular_velocity.z;

    // DriverLog("%d %f %f %f", relhistget, pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2]);

    convert_quaternion(wrist_pose_in_global.pose.orientation, pose.qRotation);

// Update the fake controller pose
update:
    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(device_id_, pose, sizeof(vr::DriverPose_t));
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