/*
 * dmr_rx - logging stub.
 *
 * This file is NOT part of MMDVMHost; it replaces upstream Log.h (which pulls
 * in std::string and nlohmann/json) with no-op functions.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef DMR_RX_MMDVM_LOG_SHIM_H
#define DMR_RX_MMDVM_LOG_SHIM_H

static inline void LogDebug(const char*, ...) {}
static inline void LogMessage(const char*, ...) {}
static inline void LogInfo(const char*, ...) {}
static inline void LogWarning(const char*, ...) {}
static inline void LogError(const char*, ...) {}
static inline void LogFatal(const char*, ...) {}

#endif
