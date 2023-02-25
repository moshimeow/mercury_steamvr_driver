// HEY. I'M REALLY TIRED AS I WRITE THIS. PLEASE READ IT CRITICALLY.
#include <iostream>

#include <fstream>

#include "xrt/xrt_config.h"

#include "tracking/t_hand_tracking.h"
#include "math/m_space.h"
#include "hg_interface.h"

#include "os/os_time.h"
#include "openvr.h"
#include "u_subprocess_logging.h"
#include "math/m_vec3.h"

#define meow_printf U_SP_LOG_E

static void
decide(xrt_vec3 one, xrt_vec3 two, bool *out)
{
    float dist = m_vec3_len(one - two);
    // These used to be 0.02f and 0.04f, but I bumped them way up to compensate for bad tracking. Once our tracking
    // is better, bump these back down.
    float activation_dist = 0.03f;
    float deactivation_dist = 0.06f;

    float pinch_activation_dist = activation_dist;

    if (*out)
    {
        pinch_activation_dist = deactivation_dist;
    }
    // const float pinch_activation_dist =
    //     (*out ? activation_dist : deactivation_dist);

    *out = (dist < pinch_activation_dist);

    U_SP_LOG_E("Meow! %f %d", dist, *out);
}

static void trigger_decide(struct xrt_hand_joint_set &joint_set, bool *out)
{
    xrt_vec3 ps[2] = {joint_set.values.hand_joint_set_default[XRT_HAND_JOINT_INDEX_TIP].relation.pose.position,
                      joint_set.values.hand_joint_set_default[XRT_HAND_JOINT_THUMB_TIP].relation.pose.position};

    decide(ps[0], ps[1], out);
}

static const float cm2m = 0.01f;

static xrt_pose aim_pose(int hand_idx, xrt_pose &wrist_primary, xrt_pose &index_pxm_primary, xrt_pose *wrist_secondary, xrt_pose &head)
{
    struct xrt_vec3 vec3_up = {0, 1, 0};

    // Average shoulder width for women:37cm, men:41cm, center of shoulder
    // joint is around 4cm inwards
    const float avg_shoulder_width = ((39.0f / 2.0f) - 4.0f) * cm2m;
    const float head_length = 10 * cm2m;
    const float neck_length = 7 * cm2m;

    // Chest center is down to the base of the head, and then down the neck.
    xrt_vec3 down_the_base_of_head;
    xrt_vec3 base_head_direction = {0, -head_length, 0};

    math_quat_rotate_vec3(&head.orientation, &base_head_direction, &down_the_base_of_head);

    xrt_vec3 chest_center = head.position + down_the_base_of_head + xrt_vec3{0, -neck_length, 0};

    xrt_vec3 face_fwd;
    xrt_vec3 forwards = {0, 0, -1};

    math_quat_rotate_vec3(&head.orientation, &forwards, &face_fwd);

    face_fwd = m_vec3_mul_scalar(m_vec3_normalize(face_fwd), 2);
    face_fwd += m_vec3_mul_scalar(
        m_vec3_normalize(wrist_primary.position - chest_center), 1);
    if (wrist_secondary)
    {
        face_fwd += m_vec3_mul_scalar(
            m_vec3_normalize(wrist_secondary->position - chest_center),
            1);
    }
    face_fwd.y = 0;
    m_vec3_normalize(face_fwd);

    xrt_vec3 face_right;
    math_vec3_cross(&face_fwd, &vec3_up, &face_right);
    math_vec3_normalize(&face_right);
    face_right *= avg_shoulder_width;

    xrt_vec3 shoulder = chest_center + face_right * (hand_idx == 1 ? 1.0f : -1.0f);

    xrt_vec3 ray_joint = index_pxm_primary.position;

    struct xrt_vec3 ray_direction = shoulder - ray_joint;

    struct xrt_vec3 up = {0, 1, 0};

    struct xrt_vec3 out_x_vector;

    // math_vec3_normalize(&tip_to_palm);
    math_vec3_normalize(&ray_direction);

    math_vec3_cross(&up, &ray_direction, &out_x_vector);

    out_x_vector = m_vec3_normalize(out_x_vector);

    xrt_pose out_pose = {};

    out_pose.position = ray_joint;

    math_quat_from_plus_x_z(&out_x_vector, &ray_direction, &out_pose.orientation);

    meow_printf("%f %f %f\n", head.position.y, shoulder.y, out_pose.position.y);

    return out_pose;
}