// !!!!!THIS WAS COPIED FROM MERCURY_TRAIN!!!!! Please at some point put this into Monado, or something.

#pragma once
#include <array>
#include "xrt/xrt_defines.h"


// note! in mercury_train, the 0th pose is the elbow, the 1st is the wrist, and the rest is the same.
// This is fine for us (?) because we didn't care about XRT_HAND_JOINT_PALM ever anyhow
using hand26 = std::array<xrt_pose, 26>;


void hand_curls(const hand26 &gt, std::array<float, 5> &curls_out);
