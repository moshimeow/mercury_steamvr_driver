// HEY. I'M REALLY TIRED AS I WRITE THIS. PLEASE READ IT CRITICALLY.
#pragma once

static void debounce(float value, float activate_thresh, float deactivate_thresh, bool *out_debounced)
{
    bool flip = false;

    if (deactivate_thresh < activate_thresh)
    {
        flip = true;
    }

    
    if (*out_debounced)
    {
        activate_thresh = deactivate_thresh;
    }

    if (flip) {
        *out_debounced = value > activate_thresh;
    } else {
        *out_debounced = value < activate_thresh;
    }
}
