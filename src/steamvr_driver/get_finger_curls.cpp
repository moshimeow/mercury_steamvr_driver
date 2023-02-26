// !!!!!THIS WAS COPIED FROM MERCURY_TRAIN!!!!! Please at some point put this into Monado, or something.

// !!!!NOTE!!!! THUMB IS STILL BROKEN FOR RIGHT HANDS! THIS DOESN'T MATTER BECAUSE WE DON'T USE THUMB!

#pragma once
#include "get_finger_curls.hpp"
#include "math/m_eigen_interop.hpp"

using namespace xrt::auxiliary::math;


/*!
 * Converts a quaternion to XY-swing and Z-twist
 *
 * @relates xrt_quat
 * @ingroup aux_math
 */
static float
quat2curl(Eigen::Quaternionf rot)
{
    // Eigen::Quaternionf rot = map_quat(*in);

    Eigen::Vector3f our_z = rot * (Eigen::Vector3f::UnitZ());

    Eigen::Quaternionf swing = Eigen::Quaternionf().setFromTwoVectors(Eigen::Vector3f::UnitZ(), our_z);

    Eigen::Quaternionf twist = swing.inverse() * rot;

    Eigen::AngleAxisf twist_aax = Eigen::AngleAxisf(twist);

    Eigen::AngleAxisf swing_aax = Eigen::AngleAxisf(swing);

    return swing_aax.axis().x() * swing_aax.angle();
}

static float
curl_diff(const hand26 &gt, int start_idx, Eigen::Quaternionf last_orientation)
{
    float ret = 0.0f;

    // For proximal, intermediate, distal, (tip has 0 rotation so no)
    for (int i = 0; i < 4; i++)
    {
        int pos = start_idx + i;
        Eigen::Quaternionf this_orientation = map_quat(gt[pos].orientation);

        Eigen::Quaternionf diff = last_orientation.conjugate() * this_orientation;

        float curl = quat2curl(diff);

        //U_LOG_T("Pos %d joint %d curl %f", pos, i, curl);
        ret += curl;

        last_orientation = this_orientation;
    }
    return ret;
}

static float
finger_curl_sum(const hand26 &gt, int finger_idx)
{
    int index_metacarpal = 5;
    int num_joints_in_full_finger = 5;
    int root = (index_metacarpal + (num_joints_in_full_finger * finger_idx) + 1);

    Eigen::Quaternionf last_orientation = map_quat(gt[0].orientation); // wrist joint

    //U_LOG_T("Finger %d", finger_idx);

    return curl_diff(gt, root, last_orientation);
}

static float
thumb_curl_sum(const hand26 &gt)
{
    Eigen::Quaternionf thumb_hidden_orientation = {};
    thumb_hidden_orientation.w() = 0.716990172863006591796875;
    thumb_hidden_orientation.x() = 0.1541481912136077880859375;
    thumb_hidden_orientation.y() = -0.31655871868133544921875;
    thumb_hidden_orientation.z() = -0.6016261577606201171875;

    Eigen::Quaternionf wrist = map_quat(gt[0].orientation);

    // Thumb part!
    Eigen::Quaternionf last_orientation = wrist * thumb_hidden_orientation;

    // //U_LOG_T("Thumb!");

    float ret = 0.0f;

    //     // For proximal, intermediate, distal, (tip has 0 rotation so no)
    for (int pos = XRT_HAND_JOINT_THUMB_METACARPAL; pos < XRT_HAND_JOINT_THUMB_TIP; pos++)
    {
        Eigen::Quaternionf this_orientation = map_quat(gt[pos].orientation);

        Eigen::Quaternionf diff = last_orientation.conjugate() * this_orientation;

        float curl = quat2curl(diff);

        // //U_LOG_T("Pos %d joint %d curl %f", pos, i, curl);
        ret += curl;

        last_orientation = this_orientation;
    }

    return ret;
}

void hand_curls(const hand26 &gt, std::array<float, 5> &curls_out)
{
    curls_out[0] = thumb_curl_sum(gt);
    curls_out[1] = finger_curl_sum(gt, 0);
    curls_out[2] = finger_curl_sum(gt, 1);
    curls_out[3] = finger_curl_sum(gt, 2);
    curls_out[4] = finger_curl_sum(gt, 3);
}
