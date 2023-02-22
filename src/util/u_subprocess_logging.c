// Copyright 2019-2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Logging functions.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_util
 */

#include "u_subprocess_logging.h"
#include "xrt/xrt_config_os.h"
#include "xrt/xrt_config_build.h"

#include "util/u_debug.h"

#include <assert.h>
#include <stdio.h>
#include <stdarg.h>
#include <debugapi.h>
#include <stdlib.h> // abort()


DEBUG_GET_ONCE_LOG_OPTION(global_log, "XRT_LOG", U_SP_LOGGING_WARN)

enum u_sp_logging_level
u_sp_log_get_global_level(void)
{
	return debug_get_log_option_global_log();
}

// Logging sink global data.
static u_sp_log_sink_func_t g_log_sink_func;
static void *g_log_sink_data;

static FILE* out_file = NULL;

void
u_sp_log_set_sink(u_sp_log_sink_func_t func, void *data)
{
	g_log_sink_func = func;
	g_log_sink_data = data;
}

#define DISPATCH_SINK(FILE, LINE, FUNC, LEVEL, FORMAT, ARGS)                                                           \
	if (g_log_sink_func != NULL) {                                                                                 \
		va_list copy;                                                                                          \
		va_copy(copy, ARGS);                                                                                   \
		g_log_sink_func(FILE, LINE, FUNC, LEVEL, FORMAT, copy, g_log_sink_data);                               \
		va_end(copy);                                                                                          \
	}

void handle_file() {
    out_file = fopen("C:\\dev\\debug.txt", "w");
    if (!out_file) {
        abort();
    }
}

static int
print_prefix(int remainingBuf, char *buf, const char *func, enum u_sp_logging_level level)
{
	int printed = 0;
	switch (level) {
	case U_SP_LOGGING_TRACE: printed = sprintf_s(buf, remainingBuf, "TRACE "); break;
	case U_SP_LOGGING_DEBUG: printed = sprintf_s(buf, remainingBuf, "DEBUG "); break;
	case U_SP_LOGGING_INFO: printed = sprintf_s(buf, remainingBuf, " INFO "); break;
	case U_SP_LOGGING_WARN: printed = sprintf_s(buf, remainingBuf, " WARN "); break;
	case U_SP_LOGGING_ERROR: printed = sprintf_s(buf, remainingBuf, "ERROR "); break;
	case U_SP_LOGGING_RAW: break;
	default: break;
	}

	if (level != U_SP_LOGGING_RAW && func != NULL) {
		printed += sprintf_s(buf + printed, remainingBuf - printed, "[%s] ", func);
	}
	return printed;
}





void
u_sp_log(const char *file, int line, const char *func, enum u_sp_logging_level level, const char *format, ...)
{
    if (!out_file) {
        handle_file();
    }

	char buf[16384] = {0};

	int remainingBuffer = sizeof(buf) - 2; // 2 for \n\0
	int printed = print_prefix(remainingBuffer, buf, func, level);

	va_list args;
	va_start(args, format);
	DISPATCH_SINK(file, line, func, level, format, args);
	printed += vsprintf_s(buf + printed, remainingBuffer - printed, format, args);
	va_end(args);
	buf[printed++] = '\n';
	buf[printed++] = '\0';
	OutputDebugStringA(buf);
	fprintf(stderr, "%s", buf);
    fprintf(out_file, "%s", buf);
    fflush(out_file);
}

