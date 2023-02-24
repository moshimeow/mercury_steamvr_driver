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

    if (*out) {
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