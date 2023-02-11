#include "mercury_device.h"

#include "util/driver_log.h"

MercuryHandDevice::MercuryHandDevice(vr::ETrackedControllerRole role, struct xrt_pose in_head_in_left) {
    role_ = role;

    math_pose_invert(&in_head_in_left, &this->left_camera_in_head);
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
    //vr::VRProperties()->SetInt32Property(props, vr::Prop_ControllerRoleHint_Int32, IsLeftHand() ? vr::TrackedControllerRole_RightHand : vr::TrackedControllerRole_LeftHand);
    vr::VRProperties()->SetUint64Property(props, vr::Prop_SupportedButtons_Uint64, 0);

    this->has_activated_ = true;

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

template <class T, class U>
void
convert_quaternion(const T &p_quatA, U &p_quatB)
{
	p_quatB.x = p_quatA.x;
	p_quatB.y = p_quatA.y;
	p_quatB.z = p_quatA.z;
	p_quatB.w = p_quatA.w;
}


template <class T, class U>
void
convert_vec3(const T &p_quatA, U &p_quatB)
{
	p_quatB.x = p_quatA.x;
	p_quatB.y = p_quatA.y;
	p_quatB.z = p_quatA.z;
}



// Updates the poses of the fake controllers as well as the finger poses
void MercuryHandDevice::UpdateHandTracking(const xrt_hand_joint_set *joint_set) {
    if (!this->has_activated_) return;

    DriverLog("mewo");
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


    // Calculate a new fake controller pose
    vr::DriverPose_t pose{};
    pose.qDriverFromHeadRotation.w = 1;
    pose.qWorldFromDriverRotation.w = 1;

    pose.deviceIsConnected = true;
    pose.poseIsValid = true;
    pose.poseTimeOffset = 0;
    pose.result = vr::TrackingResult_Running_OK;

    vr::TrackedDevicePose_t hmd_pose;
    vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0, &hmd_pose, 1);
    vr::HmdVector3d_t hmd_position = GetPosition(hmd_pose.mDeviceToAbsoluteTracking);
    vr::HmdQuaternion_t hmd_rotation = GetRotation(hmd_pose.mDeviceToAbsoluteTracking);

#if 0
    // vr::TrackedDevicePose_t &hmd_pose = other_poses;


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
#else
    for (int i = 0; i < 3; i++) {
        pose.vecWorldFromDriverTranslation[i] = 0.0;
        pose.vecAngularVelocity[i] = 0.0f;
        pose.vecVelocity[i] =  0.0f;
    }

    // pose.vecWorldFromDriverTranslation[0] = 0.0;
    // pose.vecWorldFromDriverTranslation[1] = 0.0;
    // pose.vecWorldFromDriverTranslation[2] = 0.0;

    pose.qWorldFromDriverRotation = {1,0,0,0};

    struct xrt_pose head_pose;
    convert_quaternion(hmd_rotation, head_pose.orientation);
    // convert_vec3(hmd_position, head_pose.position);
    head_pose.position.x = hmd_position.v[0];
    head_pose.position.y = hmd_position.v[1];
    head_pose.position.z = hmd_position.v[2];

    //! @todo Fragile! Depends on hand_joint_set_default->hand_relation being identity.
    struct xrt_pose wrist_pose_in_left_camera = joint_set->values.hand_joint_set_default[XRT_HAND_JOINT_WRIST].relation.pose;

    struct xrt_space_relation wrist_pose_in_global;
    struct xrt_relation_chain xrc = {};
    m_relation_chain_push_pose_if_not_identity(&xrc, &wrist_pose_in_left_camera);
    m_relation_chain_push_pose_if_not_identity(&xrc, &this->left_camera_in_head);
    m_relation_chain_push_pose_if_not_identity(&xrc, &head_pose);
    m_relation_chain_resolve(&xrc, &wrist_pose_in_global);

    pose.vecPosition[0] = wrist_pose_in_global.pose.position.x;
    pose.vecPosition[1] = wrist_pose_in_global.pose.position.y;
    pose.vecPosition[2] = wrist_pose_in_global.pose.position.z;
    convert_quaternion(wrist_pose_in_global.pose.orientation, pose.qRotation);
#endif

    // Update the fake controller pose
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