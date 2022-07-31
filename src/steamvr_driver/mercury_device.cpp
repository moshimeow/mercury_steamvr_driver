#include "mercury_device.h"

#include "util/driver_log.h"

MercuryHandDevice::MercuryHandDevice(vr::ETrackedControllerRole role) {
    role_ = role;
}

vr::EVRInitError MercuryHandDevice::Activate(uint32_t unObjectId) {
    device_id_ = unObjectId;

    vr::PropertyContainerHandle_t props = vr::VRProperties()->TrackedDeviceToPropertyContainer(
            device_id_);
    vr::VRProperties()->SetInt32Property(props, vr::Prop_ControllerHandSelectionPriority_Int32, 2147483647);
    vr::VRProperties()->SetStringProperty(props, vr::Prop_SerialNumber_String, GetSerialNumber().c_str());
    vr::VRProperties()->SetBoolProperty(props, vr::Prop_DeviceIsWireless_Bool, true);
    vr::VRProperties()->SetBoolProperty(props, vr::Prop_DeviceIsCharging_Bool, false);
    vr::VRProperties()->SetBoolProperty(props, vr::Prop_DeviceCanPowerOff_Bool, false);
    vr::VRProperties()->SetBoolProperty(props, vr::Prop_Identifiable_Bool, false);
    vr::VRProperties()->SetStringProperty(props, vr::Prop_ResourceRoot_String, "indexcontroller");
    vr::VRProperties()->SetStringProperty(props, vr::Prop_RegisteredDeviceType_String,
                                          IsLeftHand() ? "valve/index_controllerLHR-E217CD00"
                                                       : "valve/index_controllerLHR-E217CD01");
    vr::VRProperties()->SetStringProperty(props, vr::Prop_ControllerType_String, "knuckles");
    vr::VRProperties()->SetStringProperty(props, vr::Prop_ManufacturerName_String, "danwillm");

    vr::VRDriverInput()->CreateSkeletonComponent(
            props,
            IsLeftHand() ? "/input/skeleton/left" : "/input/skeleton/right",
            IsLeftHand() ? "/skeleton/hand/left" : "/skeleton/hand/right",
            "/pose/raw",
            vr::EVRSkeletalTrackingLevel::VRSkeletalTracking_Full,
            bone_transforms_,
            OPENVR_BONE_COUNT,
            &skeleton_component_handle_);

    has_activated_ = true;

    return vr::VRInitError_None;
}

vr::HmdVector3d_t GetPosition(const vr::HmdMatrix34_t &matrix) {
    return {matrix.m[0][3], matrix.m[1][3], matrix.m[2][3]};
}

vr::HmdQuaternion_t GetRotation(const vr::HmdMatrix34_t &matrix) {
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

void MercuryHandDevice::UpdateHandTracking(const xrt_hand_joint_set *joint_set) {
    if (!has_activated_) return;

    HandJointSetToBoneTransform(*joint_set, bone_transforms_, role_);

    vr::VRDriverInput()->UpdateSkeletonComponent(skeleton_component_handle_, vr::VRSkeletalMotionRange_WithController,
                                                 bone_transforms_, OPENVR_BONE_COUNT);
    vr::VRDriverInput()->UpdateSkeletonComponent(skeleton_component_handle_,
                                                 vr::VRSkeletalMotionRange_WithoutController, bone_transforms_,
                                                 OPENVR_BONE_COUNT);


    //then update the position...

    //yeet
    vr::DriverPose_t pose{};
    pose.qDriverFromHeadRotation.w = 1;
    pose.qWorldFromDriverRotation.w = 1;

    pose.deviceIsConnected = true;
    pose.poseIsValid = true;
    pose.poseTimeOffset = 0;
    pose.result = vr::TrackingResult_Running_OK;

    vr::TrackedDevicePose_t other_poses[1];
    vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0, other_poses, 1);

    vr::TrackedDevicePose_t &hmd_pose = other_poses[0];
    vr::HmdVector3d_t hmd_position = GetPosition(hmd_pose.mDeviceToAbsoluteTracking);
    vr::HmdQuaternion_t hmd_rotation = GetRotation(hmd_pose.mDeviceToAbsoluteTracking);

    pose.vecWorldFromDriverTranslation[0] = hmd_position.v[0];
    pose.vecWorldFromDriverTranslation[1] = hmd_position.v[1];
    pose.vecWorldFromDriverTranslation[2] = hmd_position.v[2];
    pose.qWorldFromDriverRotation = hmd_rotation;

    pose.vecPosition[0] = joint_set->values.hand_joint_set_default[0].relation.pose.position.x;
    pose.vecPosition[1] = joint_set->values.hand_joint_set_default[0].relation.pose.position.y;
    pose.vecPosition[2] = joint_set->values.hand_joint_set_default[0].relation.pose.position.z;

    //Copy other values from the hmd
    pose.vecAngularVelocity[0] = hmd_pose.vAngularVelocity.v[0];
    pose.vecAngularVelocity[1] = hmd_pose.vAngularVelocity.v[1];
    pose.vecAngularVelocity[2] = hmd_pose.vAngularVelocity.v[2];

    pose.vecVelocity[0] = hmd_pose.vVelocity.v[0];
    pose.vecVelocity[1] = hmd_pose.vVelocity.v[1];
    pose.vecVelocity[2] = hmd_pose.vVelocity.v[2];

    pose.qRotation.w = joint_set->values.hand_joint_set_default[0].relation.pose.orientation.w;
    pose.qRotation.x = joint_set->values.hand_joint_set_default[0].relation.pose.orientation.x;
    pose.qRotation.y = joint_set->values.hand_joint_set_default[0].relation.pose.orientation.y;
    pose.qRotation.z = joint_set->values.hand_joint_set_default[0].relation.pose.orientation.z;


    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(device_id_, pose, sizeof(vr::DriverPose_t));
}

//todo: what do we want to do here?
void MercuryHandDevice::EnterStandby() {}

void *MercuryHandDevice::GetComponent(const char *pchComponentNameAndVersion) {
    return nullptr;
}

//used when an application wants to debug a driver. maybe useful, probably not.
void MercuryHandDevice::DebugRequest(const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize) {
    if (unResponseBufferSize >= 1) pchResponseBuffer[0] = 0;
}


//this is basically never called. just return an empty pose.
vr::DriverPose_t MercuryHandDevice::GetPose() {
    DriverLog("GetPose called for role: %i. Returning empty pose", role_);

    vr::DriverPose_t pose{};
    return pose;
}

void MercuryHandDevice::Deactivate() {

}

std::string MercuryHandDevice::GetSerialNumber() {
    std::string base = "MERCURY1-";

    return base + (IsLeftHand() ? "LEFT" : "RIGHT");
}

bool MercuryHandDevice::IsLeftHand() {
    return role_ == vr::TrackedControllerRole_LeftHand;
}