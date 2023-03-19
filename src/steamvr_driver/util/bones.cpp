#include "bones.h"
#include "tracking/t_hand_tracking.h"
#include "math/m_api.h"
#include "math/m_vec3.h"

template <class T, class U>
void convert_quaternion(const T &p_quatA, U &p_quatB)
{
    p_quatB.x = p_quatA.x;
    p_quatB.y = p_quatA.y;
    p_quatB.z = p_quatA.z;
    p_quatB.w = p_quatA.w;
}

static xrt_quat
ApplyBoneHandTransform(xrt_quat p_rot, vr::ETrackedControllerRole role)
{
    std::swap(p_rot.x, p_rot.z);
    p_rot.z *= -1.f;
    if (role == vr::TrackedControllerRole_RightHand)
        return p_rot;

    p_rot.x *= -1.f;
    p_rot.y *= -1.f;
    return p_rot;
}

static void
MetacarpalJointsToBoneTransform(struct xrt_hand_joint_set *hand_joint_set,
                                vr::VRBoneTransform_t *out_bone_transforms,
                                vr::ETrackedControllerRole role)
{
    struct xrt_hand_joint_value *joint_values = hand_joint_set->values.hand_joint_set_default;

    // Apply orientations for four-finger metacarpals.
    for (int joint :
         {XRT_HAND_JOINT_THUMB_METACARPAL, XRT_HAND_JOINT_INDEX_METACARPAL, XRT_HAND_JOINT_MIDDLE_METACARPAL,
          XRT_HAND_JOINT_RING_METACARPAL, XRT_HAND_JOINT_LITTLE_METACARPAL})
    {
        struct xrt_hand_joint_value *current_joint = &joint_values[joint];
        struct xrt_hand_joint_value *parent_joint = &joint_values[XRT_HAND_JOINT_WRIST];

        xrt_quat diff_openxr;
        // These should do the exact same things.
        xrt_quat parent_inv;
        math_quat_invert(&parent_joint->relation.pose.orientation, &parent_inv);
        math_quat_rotate(&parent_inv, &current_joint->relation.pose.orientation, &diff_openxr);
        xrt_quat diff_openvr = ApplyBoneHandTransform(diff_openxr, role);

        /**
         * * if you try applying the metacarpal transforms without the magic quaternion, everything from the
         * metacarpals onwards is rotated 90 degrees.
         * In the neutral pose sample, all the metacarpals have a
         * rotation relatively close to {w=0.5, x=0.5, y=-0.5, z=0.5} which is an Important Quaternion because
         * it probably represents some 90 degree rotation. Maybe, and this was just a random guess, if I took
         * the regular metacarpal orientations and rotated them by that quat, everything would work.
         */
        xrt_quat magic_prerotate = XRT_QUAT_IDENTITY;
        magic_prerotate.w = 0.5;
        magic_prerotate.x = 0.5;
        magic_prerotate.y = -0.5;
        magic_prerotate.z = 0.5;

        if (role == vr::TrackedControllerRole_RightHand)
        {
            magic_prerotate.y *= -1.f;
            magic_prerotate.x *= -1.f;
        }

        xrt_quat final_diff;
        math_quat_rotate(&magic_prerotate, &diff_openvr, &final_diff);
        convert_quaternion(final_diff, out_bone_transforms[joint].orientation);

        xrt_vec3 global_diff_from_this_to_parent =
            m_vec3_sub(current_joint->relation.pose.position, parent_joint->relation.pose.position);

        xrt_vec3 translation_wrist_rel;
        math_quat_rotate_vec3(&parent_inv, &global_diff_from_this_to_parent, &translation_wrist_rel);

        // Y = X?
        out_bone_transforms[joint].position.v[0] = translation_wrist_rel.y;
        out_bone_transforms[joint].position.v[1] = translation_wrist_rel.x;
        out_bone_transforms[joint].position.v[2] = -translation_wrist_rel.z;
        out_bone_transforms[joint].position.v[3] = 1.f;

        if (role == vr::TrackedControllerRole_RightHand)
        {
            out_bone_transforms[joint].position.v[1] *= -1.f;
        }
    }
}

static void
FlexionJointsToBoneTransform(struct xrt_hand_joint_set *hand_joint_set,
                             vr::VRBoneTransform_t *out_bone_transforms,
                             vr::ETrackedControllerRole role)
{
    struct xrt_hand_joint_value *joint_values = hand_joint_set->values.hand_joint_set_default;

    // Apply orientations for four-finger pxm and onward
    int parent;
    for (int joint = XRT_HAND_JOINT_THUMB_METACARPAL; joint < XRT_HAND_JOINT_COUNT; joint++)
    {
        if (u_hand_joint_is_metacarpal((xrt_hand_joint)joint))
        {
            parent = joint;
            continue;
        }
        struct xrt_hand_joint_value *current_joint = &joint_values[joint];
        struct xrt_hand_joint_value *parent_joint = &joint_values[parent];

        xrt_quat diff_openxr;
        math_quat_unrotate(&parent_joint->relation.pose.orientation, &current_joint->relation.pose.orientation,
                           &diff_openxr);

        xrt_quat diff_openvr = ApplyBoneHandTransform(diff_openxr, role);
        convert_quaternion(diff_openvr, out_bone_transforms[joint].orientation);
        xrt_vec3 global_diff_from_this_to_parent =
            m_vec3_sub(current_joint->relation.pose.position, parent_joint->relation.pose.position);

        float bone_length = m_vec3_len(global_diff_from_this_to_parent);
        // OpenVR left hand has +X forward. Weird, huh?
        out_bone_transforms[joint].position = {bone_length, 0, 0, 1};

        if (role == vr::TrackedControllerRole_RightHand)
        {
            out_bone_transforms[joint].position.v[0] *= -1.f;
        }

        parent = joint;
    }
}

#define DEG_TO_RAD(DEG) (DEG * M_PI / 180.)

void HandJointSetToBoneTransform(struct xrt_hand_joint_set hand_joint_set,
                                 vr::VRBoneTransform_t *out_bone_transforms,
                                 vr::ETrackedControllerRole role,
                                 xrt_pose root_pose)
{
    // fill bone transforms with a default open pose to manipulate later

    	for (int i = 0; i < OPENVR_BONE_COUNT; i++) {
		out_bone_transforms[i] = role == vr::TrackedControllerRole_LeftHand ? leftOpenPose[i] : rightOpenPose[i];
	}

    vr::VRBoneTransform_t ident = {{0.000000f, 0.000000f, 0.000000f, 1.000000f}, {1.000000f, 0.000000f, 0.000000f, 0.000000f}};

    for (int i : {0, 1})
    {
        out_bone_transforms[i] = ident;
    }

    // I have no idea why these are required, nor why they are different per hand. (meowses)
    if (role == vr::TrackedControllerRole_LeftHand)
    {
        xrt_quat x_180_z_neg90 = {};

        // Don't try to make these bracket-initializers! If you switch back to HmdQuaternionf you'll be sad because they have different w-ordering
        x_180_z_neg90.w = 0.0f;
        x_180_z_neg90.x = 0.707f;
        x_180_z_neg90.y = 0.707f;
        x_180_z_neg90.z = 0.0f;

        math_quat_rotate(&root_pose.orientation, &x_180_z_neg90, &root_pose.orientation);
    }

    else
    {
        xrt_quat x_180_z_90 = {};
        x_180_z_90.w = 0.0f;
        x_180_z_90.x = 0.707f;
        x_180_z_90.y = -0.707f;
        x_180_z_90.z = 0.0f;
        math_quat_rotate(&root_pose.orientation, &x_180_z_90, &root_pose.orientation);
    }

    convert_quaternion(root_pose.orientation, out_bone_transforms[1].orientation);

    out_bone_transforms[1].position.v[0] = root_pose.position.x;
    out_bone_transforms[1].position.v[1] = root_pose.position.y;
    out_bone_transforms[1].position.v[2] = root_pose.position.z;
    out_bone_transforms[1].position.v[3] = 1.0f;

    MetacarpalJointsToBoneTransform(&hand_joint_set, out_bone_transforms, role);
    FlexionJointsToBoneTransform(&hand_joint_set, out_bone_transforms, role);
}

// these are currently only used for the root and wrist transforms, but are kept here as they are useful for debugging.
vr::VRBoneTransform_t rightOpenPose[OPENVR_BONE_COUNT] = {
    {{0.000000f, 0.000000f, 0.000000f, 1.000000f}, {1.000000f, 0.000000f, 0.000000f, 0.000000f}},    // WTF is this?
    {{0.034038f, 0.036503f, 0.164722f, 1.000000f}, {-0.055147f, -0.078608f, 0.920279f, -0.379296f}}, // WTF is this? Offset from Index "aim pose" to the wrist???
    //
    {{0.012083f, 0.028070f, 0.025050f, 1.000000f}, {0.567418f, -0.464112f, 0.623374f, -0.272106f}}, // Thumb
    {{-0.040406f, -0.000000f, 0.000000f, 1.000000f}, {0.994838f, 0.082939f, 0.019454f, 0.055130f}},
    {{-0.032517f, -0.000000f, -0.000000f, 1.000000f}, {0.974793f, -0.003213f, 0.021867f, -0.222015f}},
    {{-0.030464f, 0.000000f, 0.000000f, 1.000000f}, {1.000000f, -0.000000f, -0.000000f, 0.000000f}},
    //
    {{-0.000632f, 0.026866f, 0.015002f, 1.000000f}, {0.421979f, -0.644251f, 0.422133f, 0.478202f}}, // Index
    {{-0.074204f, 0.005002f, -0.000234f, 1.000000f}, {0.995332f, 0.007007f, -0.039124f, 0.087949f}},
    {{-0.043930f, 0.000000f, 0.000000f, 1.000000f}, {0.997891f, 0.045808f, 0.002142f, -0.045943f}},
    {{-0.028695f, -0.000000f, -0.000000f, 1.000000f}, {0.999649f, 0.001850f, -0.022782f, -0.013409f}},
    {{-0.022821f, -0.000000f, 0.000000f, 1.000000f}, {1.000000f, -0.000000f, 0.000000f, -0.000000f}},
    //
    {{-0.002177f, 0.007120f, 0.016319f, 1.000000f}, {0.541276f, -0.546723f, 0.460749f, 0.442520f}}, // Middle
    {{-0.070953f, -0.000779f, -0.000997f, 1.000000f}, {0.980294f, -0.167261f, -0.078959f, 0.069368f}},
    {{-0.043108f, -0.000000f, -0.000000f, 1.000000f}, {0.997947f, 0.018493f, 0.013192f, 0.059886f}},
    {{-0.033266f, -0.000000f, -0.000000f, 1.000000f}, {0.997394f, -0.003328f, -0.028225f, -0.066315f}},
    {{-0.025892f, 0.000000f, -0.000000f, 1.000000f}, {0.999195f, -0.000000f, 0.000000f, 0.040126f}},
    //
    {{-0.000513f, -0.006545f, 0.016348f, 1.000000f}, {0.550143f, -0.516692f, 0.429888f, 0.495548f}}, // Ring
    {{-0.065876f, -0.001786f, -0.000693f, 1.000000f}, {0.990420f, -0.058696f, -0.101820f, 0.072495f}},
    {{-0.040697f, -0.000000f, -0.000000f, 1.000000f}, {0.999545f, -0.002240f, 0.000004f, 0.030081f}},
    {{-0.028747f, 0.000000f, 0.000000f, 1.000000f}, {0.999102f, -0.000721f, -0.012693f, 0.040420f}},
    {{-0.022430f, 0.000000f, -0.000000f, 1.000000f}, {1.000000f, 0.000000f, 0.000000f, 0.000000f}},

    {{0.002478f, -0.018981f, 0.015214f, 1.000000f}, {0.523940f, -0.526918f, 0.326740f, 0.584025f}}, // Pinky
    {{-0.062878f, -0.002844f, -0.000332f, 1.000000f}, {0.986609f, -0.059615f, -0.135163f, 0.069132f}},
    {{-0.030220f, -0.000000f, -0.000000f, 1.000000f}, {0.994317f, 0.001896f, -0.000132f, 0.106446f}},
    {{-0.018187f, -0.000000f, -0.000000f, 1.000000f}, {0.995931f, -0.002010f, -0.052079f, -0.073526f}},
    {{-0.018018f, -0.000000f, 0.000000f, 1.000000f}, {1.000000f, 0.000000f, 0.000000f, 0.000000f}},

    {{0.006059f, 0.056285f, 0.060064f, 1.000000f}, {0.737238f, 0.202745f, -0.594267f, -0.249441f}}, // Aux
    {{0.040416f, -0.043018f, 0.019345f, 1.000000f}, {-0.290331f, 0.623527f, 0.663809f, 0.293734f}},
    {{0.039354f, -0.075674f, 0.047048f, 1.000000f}, {-0.187047f, 0.678062f, 0.659285f, 0.265683f}},
    {{0.038340f, -0.090987f, 0.082579f, 1.000000f}, {-0.183037f, 0.736793f, 0.634757f, 0.143936f}},
    {{0.031806f, -0.087214f, 0.121015f, 1.000000f}, {-0.003659f, 0.758407f, 0.639342f, 0.126678f}},
};

vr::VRBoneTransform_t leftOpenPose[OPENVR_BONE_COUNT] = {
    {{0.000000f, 0.000000f, 0.000000f, 1.000000f}, {1.000000f, 0.000000f, 0.000000f, 0.000000f}}, // Root
    //
    {{-0.034038f, 0.036503f, 0.164722f, 1.000000f}, {-0.055147f, -0.078608f, -0.920279f, 0.379296f}}, // Thumb
    {{-0.012083f, 0.028070f, 0.025050f, 1.000000f}, {0.464112f, 0.567418f, 0.272106f, 0.623374f}},
    {{0.040406f, 0.000000f, -0.000000f, 1.000000f}, {0.994838f, 0.082939f, 0.019454f, 0.055130f}},
    {{0.032517f, 0.000000f, 0.000000f, 1.000000f}, {0.974793f, -0.003213f, 0.021867f, -0.222015f}},
    {{0.030464f, -0.000000f, -0.000000f, 1.000000f}, {1.000000f, -0.000000f, -0.000000f, 0.000000f}},
    //
    {{0.000632f, 0.026866f, 0.015002f, 1.000000f}, {0.644251f, 0.421979f, -0.478202f, 0.422133f}}, // Index
    {{0.074204f, -0.005002f, 0.000234f, 1.000000f}, {0.995332f, 0.007007f, -0.039124f, 0.087949f}},
    {{0.043930f, -0.000000f, -0.000000f, 1.000000f}, {0.997891f, 0.045808f, 0.002142f, -0.045943f}},
    {{0.028695f, 0.000000f, 0.000000f, 1.000000f}, {0.999649f, 0.001850f, -0.022782f, -0.013409f}},
    {{0.022821f, 0.000000f, -0.000000f, 1.000000f}, {1.000000f, -0.000000f, 0.000000f, -0.000000f}},
    //
    {{0.002177f, 0.007120f, 0.016319f, 1.000000f}, {0.546723f, 0.541276f, -0.442520f, 0.460749f}}, // Middle
    {{0.070953f, 0.000779f, 0.000997f, 1.000000f}, {0.980294f, -0.167261f, -0.078959f, 0.069368f}},
    {{0.043108f, 0.000000f, 0.000000f, 1.000000f}, {0.997947f, 0.018493f, 0.013192f, 0.059886f}},
    {{0.033266f, 0.000000f, 0.000000f, 1.000000f}, {0.997394f, -0.003328f, -0.028225f, -0.066315f}},
    {{0.025892f, -0.000000f, 0.000000f, 1.000000f}, {0.999195f, -0.000000f, 0.000000f, 0.040126f}},

    {{0.000513f, -0.006545f, 0.016348f, 1.000000f}, {0.516692f, 0.550143f, -0.495548f, 0.429888f}}, // Ring
    {{0.065876f, 0.001786f, 0.000693f, 1.000000f}, {0.990420f, -0.058696f, -0.101820f, 0.072495f}},
    {{0.040697f, 0.000000f, 0.000000f, 1.000000f}, {0.999545f, -0.002240f, 0.000004f, 0.030081f}},
    {{0.028747f, -0.000000f, -0.000000f, 1.000000f}, {0.999102f, -0.000721f, -0.012693f, 0.040420f}},
    {{0.022430f, -0.000000f, 0.000000f, 1.000000f}, {1.000000f, 0.000000f, 0.000000f, 0.000000f}},

    {{-0.002478f, -0.018981f, 0.015214f, 1.000000f}, {0.526918f, 0.523940f, -0.584025f, 0.326740f}}, // Pinky
    {{0.062878f, 0.002844f, 0.000332f, 1.000000f}, {0.986609f, -0.059615f, -0.135163f, 0.069132f}},
    {{0.030220f, 0.000000f, 0.000000f, 1.000000f}, {0.994317f, 0.001896f, -0.000132f, 0.106446f}},
    {{0.018187f, 0.000000f, 0.000000f, 1.000000f}, {0.995931f, -0.002010f, -0.052079f, -0.073526f}},
    {{0.018018f, 0.000000f, -0.000000f, 1.000000f}, {1.000000f, 0.000000f, 0.000000f, 0.000000f}},

    {{-0.006059f, 0.056285f, 0.060064f, 1.000000f}, {0.737238f, 0.202745f, 0.594267f, 0.249441f}}, // Aux
    {{-0.040416f, -0.043018f, 0.019345f, 1.000000f}, {-0.290331f, 0.623527f, -0.663809f, -0.293734f}},
    {{-0.039354f, -0.075674f, 0.047048f, 1.000000f}, {-0.187047f, 0.678062f, -0.659285f, -0.265683f}},
    {{-0.038340f, -0.090987f, 0.082579f, 1.000000f}, {-0.183037f, 0.736793f, -0.634757f, -0.143936f}},
    {{-0.031806f, -0.087214f, 0.121015f, 1.000000f}, {-0.003659f, 0.758407f, -0.639342f, -0.126678f}},
};
