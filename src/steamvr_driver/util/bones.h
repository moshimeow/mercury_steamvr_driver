#pragma once

#include "openvr_driver.h"
#include "xrt/xrt_defines.h"

#define OPENVR_BONE_COUNT 31

extern vr::VRBoneTransform_t rightOpenPose[OPENVR_BONE_COUNT];
extern vr::VRBoneTransform_t leftOpenPose[OPENVR_BONE_COUNT];

void HandJointSetToBoneTransform(struct xrt_hand_joint_set hand_joint_set,
                                 vr::VRBoneTransform_t *out_bone_transforms,
                                 vr::ETrackedControllerRole role,
                                 xrt_pose root_pose = XRT_POSE_IDENTITY);
