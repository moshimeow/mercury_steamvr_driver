#pragma once

#include "xrt/xrt_defines.h"
#include <string>

// We're not using xrt_hand_joint_set because it has relation flags, acceleration, etc.
// which take up space, and we're sending these messages at 54Hz.
#pragma pack(push, 1)

struct emulated_buttons_state
{
    bool a;
    bool b;
    bool trigger;
    bool grip; // Not emulated, but you can get at it through the UI
    
    bool system; // ditto


    uint8_t _pad[2];

    bool thumbstick_gesture;


    float thumbstick_x;
    float thumbstick_y; // Not emulated, but you can get at it through UI

    //
    float curls[5];
};

struct tracking_message_hand
{
    bool tracked;
    uint8_t _pad[3];
    struct emulated_buttons_state bs;
    xrt_pose pose_raw;
    xrt_pose wrist;
    xrt_pose fingers_relative[26];
};

struct tracking_message
{
    uint32_t size;
    int64_t camera_timestamp;
    int64_t host_recieved_frame_timestamp;
    int64_t sent_at_timestamp;
    tracking_message_hand hands[2];
};

#pragma pack(pop)

#define TMSIZE sizeof(tracking_message)
static_assert(sizeof(struct tracking_message) == 1676);

static_assert(sizeof(bool) == 1);

#define TM_FMT(message) "Size: %zu, tracked %d, timestamp %zu, wrist %f %f %f, %f %f %f %f\n", message.size, message.hands[0].tracked, message.camera_timestamp, \
                        message.hands[0].wrist.position.x, message.hands[0].wrist.position.y, message.hands[0].wrist.position.z,                                 \
                        message.hands[0].wrist.orientation.w, message.hands[0].wrist.orientation.x, message.hands[0].wrist.orientation.y, message.hands[0].wrist.orientation.z
