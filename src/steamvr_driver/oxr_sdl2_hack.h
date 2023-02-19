// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Hacky SDL integration
 * @author Jakob Bornecrantz <jakob@collabora.com>
 */

#include "util/u_file.h"
#include "xrt/xrt_compiler.h"
#include "xrt/xrt_instance.h"
#include "xrt/xrt_config_have.h"
#include "xrt/xrt_config_drivers.h"

#include "util/u_var.h"
#include "util/u_misc.h"
#include "util/u_debug.h"

#include "os/os_threading.h"

#ifdef __cplusplus
extern "C" {
#endif

int
oxr_sdl2_hack_create(void **out_hack);

void
oxr_sdl2_hack_start(void *hack, struct xrt_instance *xinst, struct xrt_system_devices *xsysd);

void
oxr_sdl2_hack_stop(void **hack);

#ifdef __cplusplus
}
#endif