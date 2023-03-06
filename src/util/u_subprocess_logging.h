// Copyright 2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Basic logging functionality.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_log
 */


#pragma once

#include "xrt/xrt_compiler.h"

#include <stdarg.h>


#ifdef __cplusplus
extern "C" {
#endif



enum u_sp_logging_level
{
	U_SP_LOGGING_TRACE, //!< Trace messages, highly verbose.
	U_SP_LOGGING_DEBUG, //!< Debug messages, verbose.
	U_SP_LOGGING_INFO,  //!< Info messages: not very verbose, not indicating a problem.
	U_SP_LOGGING_WARN,  //!< Warning messages: indicating a potential problem
	U_SP_LOGGING_ERROR, //!< Error messages: indicating a problem
	U_SP_LOGGING_RAW,   //!< Special level for raw printing, prints a new-line.
};



/*!
 * Function typedef for setting the logging sink.
 *
 * @param file   Source file name associated with a message.
 * @param line   Source file line associated with a message.
 * @param func   Function name associated with a message.
 * @param level  Message level: used for formatting or forwarding to native log functions.
 * @param format Format string.
 * @param args   Format parameters.
 * @param data   User data.
 */
typedef void (*u_sp_log_sink_func_t)(const char *file,
                                  int line,
                                  const char *func,
                                  enum u_sp_logging_level level,
                                  const char *format,
                                  va_list args,
                                  void *data);

/*!
 * For places where you really want printf, prints a new-line.
 */
#define U_SP_LOG_RAW(...)                                                                                                 \
	do {                                                                                                           \
		u_sp_log(__FILE__, __LINE__, __func__, U_SP_LOGGING_RAW, __VA_ARGS__);                                       \
	} while (false)

/*!
 * @name Base Logging Utilities
 * In most cases, you will want to use another macro from this file, or a module/driver-local macro, to do your logging.
 * @{
 */
/*!
 * @brief Log a message at @p level , with file, line, and function context (always logs) - typically wrapped
 * in a helper macro.
 *
 * @param level A @ref u_sp_logging_level value for this message.
 * @param ... Format string and optional format arguments.
 */
// #define U_SP_LOG(level, ...)                                                                                              \
// 	do {                                                                                                           \
// 		u_sp_log(__FILE__, __LINE__, __func__, level, __VA_ARGS__);                                               \
// 	} while (false)

/*!
 * @brief Log at @p level only if the level is at least @p cond_level - typically wrapped in a helper macro.
 *
 * Adds file, line, and function context. Like U_SP_LOG() but conditional.
 *
 * @param level A @ref u_sp_logging_level value for this message.
 * @param cond_level The minimum @ref u_sp_logging_level that will be actually output.
 * @param ... Format string and optional format arguments.
 */
#define U_SP_LOG_IFL(level, cond_level, ...)                                                                              \
	do {                                                                                                           \
		if (cond_level <= level) {                                                                             \
			u_sp_log(__FILE__, __LINE__, __func__, level, __VA_ARGS__);                                       \
		}                                                                                                      \
	} while (false)


/*!
 * Returns the global logging level, subsystems own logging level take precedence.
 */
enum u_sp_logging_level
u_sp_log_get_global_level(void);

/*!
 * @brief Main non-device-related log implementation function: do not call directly, use a macro that wraps it.
 *
 * This function always logs: level is used for printing or passed to native logging functions.
 *
 * @param file Source file name associated with a message
 * @param line Source file line associated with a message
 * @param func Function name associated with a message
 * @param level Message level: used for formatting or forwarding to native log functions
 * @param format Format string
 * @param ... Format parameters
 */
void
u_sp_log(const char *file, int line, const char *func, enum u_sp_logging_level level, const char *format, ...)
    XRT_PRINTF_FORMAT(5, 6);

/*!
 * Sets the path to the log file
 *
 * @param path Output file to log to
*/
void u_sp_log_set_file_path(char* path);

/*!
 * Sets the logging sink, log is still passed on to the platform defined output
 * as well as the sink.
 *
 * @param func Logging function for the calls to be sent to.
 * @param data User data to be passed into @p func.
 */
void
u_sp_log_set_sink(u_sp_log_sink_func_t func, void *data);

/*!
 * @}
 */


/*!
 * @name Logging macros conditional on global log level
 *
 * These each imply a log level, and will only log if the global log level is equal or lower.
 * They are often used for one-off logging in a module with few other logging needs,
 * where having a module-specific log level would be unnecessary.
 *
 * @see U_SP_LOG_IFL, u_sp_log_get_global_level()
 * @param ... Format string and optional format arguments.
 * @{
 */
//! Log a message at U_SP_LOGGING_TRACE level, conditional on the global log level
#define U_SP_LOG_T(...) U_SP_LOG_IFL_T(u_sp_log_get_global_level(), __VA_ARGS__)

//! Log a message at U_SP_LOGGING_DEBUG level, conditional on the global log level
#define U_SP_LOG_D(...) U_SP_LOG_IFL_D(u_sp_log_get_global_level(), __VA_ARGS__)

//! Log a message at U_SP_LOGGING_INFO level, conditional on the global log level
#define U_SP_LOG_I(...) U_SP_LOG_IFL_I(u_sp_log_get_global_level(), __VA_ARGS__)

//! Log a message at U_SP_LOGGING_WARN level, conditional on the global log level
#define U_SP_LOG_W(...) U_SP_LOG_IFL_W(u_sp_log_get_global_level(), __VA_ARGS__)

//! Log a message at U_SP_LOGGING_ERROR level, conditional on the global log level
#define U_SP_LOG_E(...) U_SP_LOG_IFL_E(u_sp_log_get_global_level(), __VA_ARGS__)

//! Conditionally log a message at U_SP_LOGGING_TRACE level.
#define U_SP_LOG_IFL_T(cond_level, ...) U_SP_LOG_IFL(U_SP_LOGGING_TRACE, cond_level, __VA_ARGS__)
//! Conditionally log a message at U_SP_LOGGING_DEBUG level.
#define U_SP_LOG_IFL_D(cond_level, ...) U_SP_LOG_IFL(U_SP_LOGGING_DEBUG, cond_level, __VA_ARGS__)
//! Conditionally log a message at U_SP_LOGGING_INFO level.
#define U_SP_LOG_IFL_I(cond_level, ...) U_SP_LOG_IFL(U_SP_LOGGING_INFO, cond_level, __VA_ARGS__)
//! Conditionally log a message at U_SP_LOGGING_WARN level.
#define U_SP_LOG_IFL_W(cond_level, ...) U_SP_LOG_IFL(U_SP_LOGGING_WARN, cond_level, __VA_ARGS__)
//! Conditionally log a message at U_SP_LOGGING_ERROR level.
#define U_SP_LOG_IFL_E(cond_level, ...) U_SP_LOG_IFL(U_SP_LOGGING_ERROR, cond_level, __VA_ARGS__)
/*!
 * @}
 */


#ifdef __cplusplus
}
#endif
