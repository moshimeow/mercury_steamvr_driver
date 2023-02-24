#pragma once

#include "xrt/xrt_defines.h"
#include <string>

// We're not using xrt_hand_joint_set because it has relation flags, acceleration, etc.
// which take up space, and we're sending these messages at 54Hz.
#pragma pack(push, 1)
struct tracking_message_hand
{
    bool tracked;
    xrt_pose wrist;
    xrt_pose fingers_relative[26];
};

struct tracking_message
{
    size_t size;
    int64_t camera_timestamp;
    int64_t host_recieved_frame_timestamp;
    int64_t sent_at_timestamp;
    tracking_message_hand hands[2];
};

struct server_control_message
{
    bool quit = false;
};

#pragma pack(pop)

#define TMSIZE sizeof(tracking_message)

#define TM_FMT(message) "Size: %zu, tracked %d, timestamp %zu, wrist %f %f %f, %f %f %f %f\n", message.size, message.hands[0].tracked, message.timestamp, \
                        message.hands[0].wrist.position.x, message.hands[0].wrist.position.y, message.hands[0].wrist.position.z,                 \
                        message.hands[0].wrist.orientation.w, message.hands[0].wrist.orientation.x, message.hands[0].wrist.orientation.y, message.hands[0].wrist.orientation.z
