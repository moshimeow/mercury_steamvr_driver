// HEY. I'M REALLY TIRED AS I WRITE THIS. PLEASE READ IT CRITICALLY.

#include "everything_else_decider.hpp"
#include "get_finger_curls.hpp"

// #include "xrt/xrt_defines.h"
#include "math/m_eigen_interop.hpp"
#include "u_subprocess_logging.h"
#include "debouncer.hpp"
#include "u_subprocess_logging.h"

#define meow_printf U_SP_LOG_E

//!@todo I hate to write global state in but I'm on a huge time crunch and don't have external reviewers. Please make this more reasonable.
// struct decider_global_state
// {
//     bool snap_up = {};
//     bool snap_down = {};

//     bool curled[2][4] = {};
// };

struct decider_global_state dgs;

struct everything_else_decider
{
    tracking_message &base;
    xrt_pose hands_head_local[2];

    everything_else_decider(tracking_message &msg) : base(msg) {}
};

inline static xrt_quat thumbstick_left_hand_up_pose()
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

inline static xrt_quat thumbstick_right_hand_left_pose()
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

inline xrt_quat thumbstick_left_hand_right_pose()
{
    xrt_quat up_pose;
    up_pose.w = 0.5;
    up_pose.x = -0.5;
    up_pose.y = -0.5;
    up_pose.z = 0.5;

    return up_pose;
}

float quat_difference(xrt_quat q1, xrt_quat q2)
{
    // https://math.stackexchange.com/a/90098
    // d(q1,q2)=1−⟨q1,q2⟩2

    float inner_product = (q1.w * q2.w) + (q1.x * q2.x) + (q1.y * q2.y) + (q1.z * q2.z);
    return 1.0 - (inner_product * inner_product);
}

inline static bool both_hands_good(xrt_pose hand0, xrt_pose hand1, xrt_quat hand0_target, xrt_quat hand1_target)
{

    float left = quat_difference(hand0.orientation, hand0_target);
    float right = quat_difference(hand1.orientation, hand1_target);

    // Closer to 0 is better
    bool left_ = left < 0.25;
    bool right_ = right < 0.25;

    return (left_ && right_);
}

float hand_height(xrt_pose hand0, xrt_pose hand1)
{
    return hand1.position.y - (hand0.position.y + 0.03);
}

float hand_x(xrt_pose hand0, xrt_pose hand1)
{
    return hand1.position.x - (hand0.position.x + 0.36);
}

float hand_dist(xrt_pose hand0, xrt_pose hand1)
{
    return hand1.position.z - hand0.position.z;
}

void thumbstick(xrt_pose hand0, xrt_pose hand1, float &thumbstick_value_y, float &thumbstick_value_x, bool &thumbstick_gesture)
{

    if (both_hands_good(hand0, hand1, thumbstick_left_hand_up_pose(), thumbstick_right_hand_left_pose()))
    {
        thumbstick_gesture = true;
        {
            thumbstick_value_y = hand_dist(hand0, hand1) * 12;

            // I think this deadzone stuff is good
            float deadzone_value = 0.2f;

            if (thumbstick_value_y > 0)
            {
                thumbstick_value_y = fmaxf(0, thumbstick_value_y - deadzone_value);
            }
            else
            {
                thumbstick_value_y = fminf(0, thumbstick_value_y + deadzone_value);
            }
        }

        {
            thumbstick_value_x = -hand_x(hand0, hand1);

            meow_printf("%f", thumbstick_value_x);

            // I think this deadzone stuff is good
            float deadzone_value = 0.0125;

            if (thumbstick_value_x > 0)
            {
                thumbstick_value_x = fmaxf(0, thumbstick_value_x - deadzone_value);
            }
            else
            {
                thumbstick_value_x = fminf(0, thumbstick_value_x + deadzone_value);
            }

            thumbstick_value_x *= 8;
            meow_printf("%f", thumbstick_value_x);
        }

        return;
    }

    thumbstick_value_x = 0.0f;
    thumbstick_value_y = 0.0f;
    thumbstick_gesture = false;
}

void thumbstick_turn(xrt_pose hand0, xrt_pose hand1, float &thumbstick_value, bool &thumbstick_gesture)
{

    if (both_hands_good(hand0, hand1, thumbstick_left_hand_up_pose(), thumbstick_left_hand_right_pose()))
    {
        float val = -hand_height(hand0, hand1) * 10;

        debounce(val, 0.9, 0.3, &dgs.snap_up);
        debounce(val, -0.9, -0.3, &dgs.snap_down);

        if (dgs.snap_down)
        {
            thumbstick_value = 1.0f;
        }
        else if (dgs.snap_up)
        {
            thumbstick_value = -1.0f;
        }
        else
        {
            goto undetected;
        }

        thumbstick_gesture = true;

        return;
    }

undetected:
    thumbstick_value = 0.0f;
    thumbstick_gesture = false;
}

void curls(everything_else_decider &dec, int hand_idx)
{
    hand26 hand;
    for (int i = 0; i < XRT_HAND_JOINT_COUNT; i++)
    {
        hand[i] = dec.base.hands[hand_idx].fingers_relative[i];
    }

    std::array<float, 5> curls;
    hand_curls(hand, curls);

    for (int i = 0; i < 5; i++)
    {
        dec.base.hands[hand_idx].bs.curls[i] = curls[i];

        if (i == 0)
        {
            continue;
        }
        float thresh_uncurled = -1.0f;
        float thresh_curled = -2.0f;
        debounce(curls[i], thresh_curled, thresh_uncurled, &dgs.curled[hand_idx][i - 1]);
    }
}

// Fuuuuuck, we need debouncing, fuuuuuck, nooooo
void a(everything_else_decider &dec, int hand_idx)
{
    // A is the "horns" gesture

    bool good = !dgs.curled[hand_idx][0] && //
                dgs.curled[hand_idx][1] &&  //
                dgs.curled[hand_idx][2] &&  //
                !dgs.curled[hand_idx][3];
    if (good)
    {
        dec.base.hands[hand_idx].bs.a = true;
    }
    else
    {
        dec.base.hands[hand_idx].bs.a = false;
    }
}

void b(everything_else_decider &dec, int hand_idx)
{
    // B is: All fingers down except for pinky

    bool good = dgs.curled[hand_idx][0] && //
                dgs.curled[hand_idx][1] && //
                dgs.curled[hand_idx][2] && //
                !dgs.curled[hand_idx][3];

    if (good)
    {
        dec.base.hands[hand_idx].bs.b = true;
    }
    else
    {
        dec.base.hands[hand_idx].bs.b = false;
    }
}

void system_button(everything_else_decider &dec)
{
    // System: Peace sign with both hands. This is how you peace out!

    bool good = true;

    for (int hand_idx = 0; hand_idx < 2; hand_idx++)
    {
        good = good && !dgs.curled[hand_idx][0];
        good = good && !dgs.curled[hand_idx][1];
        good = good && dgs.curled[hand_idx][2];
        good = good && dgs.curled[hand_idx][3];
    }

    if (good)
    {
        dec.base.hands[0].bs.system = true;
    }
    else
    {
        dec.base.hands[0].bs.system = false;
    }
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
        curls(dec, i);
        if (msg.hands[i].bs.trigger)
        {
            msg.hands[i].bs.a = false;
            msg.hands[i].bs.b = false;
        }
        else
        {
            a(dec, i);
            b(dec, i);
        }
    }

    if (msg.hands[0].tracked && msg.hands[1].tracked)
    {
        thumbstick(dec.hands_head_local[0], dec.hands_head_local[1], dec.base.hands[0].bs.thumbstick_y, dec.base.hands[0].bs.thumbstick_x, dec.base.hands[0].bs.thumbstick_gesture);
        thumbstick_turn(dec.hands_head_local[1], dec.hands_head_local[0], dec.base.hands[1].bs.thumbstick_x, dec.base.hands[1].bs.thumbstick_gesture);
    }

    system_button(dec);

    // meow_printf("%f", quat_difference(dec.hands_head_local[0].orientation, up_pose));
    // meow_printf("%f", quat_difference(up_pose_2, up_pose));

    // meow_printf("%f %f %f %f, %f %f %f %f", dec.hands_head_local[0].orientation.w, dec.hands_head_local[0].orientation.x, dec.hands_head_local[0].orientation.y, dec.hands_head_local[0].orientation.z,
    //             up_pose_2.w, up_pose_2.x, up_pose_2.y, up_pose_2.z);

    // meow_printf("%f %f %f %f, %f %f %f %f", dec.hands_head_local[0].orientation.w, dec.hands_head_local[0].orientation.x, dec.hands_head_local[0].orientation.y, dec.hands_head_local[0].orientation.z,
    //             dec.hands_head_local[1].orientation.w, dec.hands_head_local[1].orientation.x, dec.hands_head_local[1].orientation.y, dec.hands_head_local[1].orientation.z);
}
