// HEY. I'M REALLY TIRED AS I WRITE THIS. PLEASE READ IT CRITICALLY.

#include "everything_else_decider.hpp"
#include "get_finger_curls.hpp"

// #include "xrt/xrt_defines.h"
#include "math/m_eigen_interop.hpp"
#include "u_subprocess_logging.h"

#define meow_printf U_SP_LOG_E

struct everything_else_decider
{
    tracking_message &base;
    xrt_pose hands_head_local[2];

    everything_else_decider(tracking_message &msg) : base(msg) {}
};

inline xrt_quat thumbstick_left_hand_up_pose()
{
#if 1
    xrt_quat up_pose;
    up_pose.w = 0.707;
    up_pose.x = 0.707;
    up_pose.y = 0;
    up_pose.z = 0;

#else
    xrt_quat up_pose_2;

    xrt_vec3 plusx = XRT_VEC3_UNIT_X;

    xrt_vec3 plusz = XRT_VEC3_UNIT_Y;

    math_quat_from_plus_x_z(&plusx, &plusz, &up_pose_2);
#endif
    return up_pose;
}

inline xrt_quat thumbstick_right_hand_left_pose()
{
#if 1
    xrt_quat up_pose;
    up_pose.w = 0.5;
    up_pose.x = -0.5;
    up_pose.y = 0.5;
    up_pose.z = -0.5;

#else
    xrt_quat up_pose_2;

    xrt_vec3 plusx = XRT_VEC3_UNIT_X;

    xrt_vec3 plusz = XRT_VEC3_UNIT_Y;

    math_quat_from_plus_x_z(&plusx, &plusz, &up_pose_2);
#endif
    return up_pose;
}

float quat_difference(xrt_quat q1, xrt_quat q2)
{
    // https://math.stackexchange.com/a/90098
    // d(q1,q2)=1−⟨q1,q2⟩2

    float inner_product = (q1.w * q2.w) + (q1.x * q2.x) + (q1.y * q2.y) + (q1.z * q2.z);
    return 1.0 - (inner_product * inner_product);
}

void thumbstick(everything_else_decider &dec)
{
    if (!(dec.base.hands[0].tracked && dec.base.hands[1].tracked))
    {
        dec.base.hands[0].bs.thumbstick_gesture = false;
        return;
    }
    float left = quat_difference(dec.hands_head_local[0].orientation, thumbstick_left_hand_up_pose());
    float right = quat_difference(dec.hands_head_local[1].orientation, thumbstick_right_hand_left_pose());

    // Closer to 0 is better
    bool left_ = left < 0.25;
    bool right_ = right < 0.25;

    meow_printf("%d %d, %f %f", left_, right_, left, right);
    meow_printf(" I am this file!!!");

    if (left_ && right_)
    {
        dec.base.hands[0].bs.thumbstick_gesture = true;

        dec.base.hands[0].bs.thumbstick_y = (dec.hands_head_local[0].position.z - dec.hands_head_local[1].position.z) * 12;

        // Very shitty deadzone. This isn't how you should do it.
        if (fabsf(dec.base.hands[0].bs.thumbstick_y) < 0.1)
        {
            dec.base.hands[0].bs.thumbstick_y = 0;
        }

        float left_h = dec.hands_head_local[0].position.y + 0.04;
        float right_h = dec.hands_head_local[1].position.y;

        float ydiff = left_h - right_h;

        if (fabsf(ydiff) > 0.1)
        {
            // Yes, right hand controller. This is what works with my VRChat profile.
            if (left_h > right_h)
            {
                dec.base.hands[1].bs.thumbstick_x = 1.0f;
            }
            else
            {
                dec.base.hands[1].bs.thumbstick_x = -1.0f;
            }
            dec.base.hands[1].bs.thumbstick_gesture = true;
        }
        else
        {
            dec.base.hands[1].bs.thumbstick_x = 0.0f;
            dec.base.hands[1].bs.thumbstick_gesture = false;
        }
    }
    else
    {
        dec.base.hands[0].bs.thumbstick_gesture = false;
    }
}

void curls(everything_else_decider &dec, int hand_idx)
{
    // hand26 hand;
    // for (int i = 0; i < XRT_HAND_JOINT_COUNT; i++) {
    //     hand[i] = dec.base.hands[hand_idx].fingers_relative[i];
    // }

    // std::array <float, 5> curls;
    // hand_curls()
}

void decide_everything_else(tracking_message &msg, xrt_pose head)
{

    // left: 0.869860 0.472481 -0.077124 -0.118981,

    xrt_quat up_pose = thumbstick_left_hand_up_pose();

    everything_else_decider dec(msg);

    for (int i = 0; i < 2; i++)
    {
        if (!msg.hands[i].tracked)
        {
            continue;
        }
        struct xrt_relation_chain xrc = {};
        xrt_space_relation tmp = {};
        m_relation_chain_push_pose(&xrc, &msg.hands[i].wrist);
        m_relation_chain_push_inverted_pose_if_not_identity(&xrc, &head);
        m_relation_chain_resolve(&xrc, &tmp);

        dec.hands_head_local[i] = tmp.pose;
    }

    thumbstick(dec);

    // meow_printf("%f", quat_difference(dec.hands_head_local[0].orientation, up_pose));
    // meow_printf("%f", quat_difference(up_pose_2, up_pose));

    // meow_printf("%f %f %f %f, %f %f %f %f", dec.hands_head_local[0].orientation.w, dec.hands_head_local[0].orientation.x, dec.hands_head_local[0].orientation.y, dec.hands_head_local[0].orientation.z,
    //             up_pose_2.w, up_pose_2.x, up_pose_2.y, up_pose_2.z);

    meow_printf("%f %f %f %f, %f %f %f %f", dec.hands_head_local[0].orientation.w, dec.hands_head_local[0].orientation.x, dec.hands_head_local[0].orientation.y, dec.hands_head_local[0].orientation.z,
                dec.hands_head_local[1].orientation.w, dec.hands_head_local[1].orientation.x, dec.hands_head_local[1].orientation.y, dec.hands_head_local[1].orientation.z);
}
