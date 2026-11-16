/*
 * dmr_rx - bit packing helpers.
 *
 * This file is NOT part of MMDVMHost. Upstream Utils.h/Utils.cpp are licensed
 * "GPL version 2 only", which cannot be combined with an AGPL-3.0 application,
 * so they are not vendored. This is an independent, minimal replacement that
 * only provides the two bit <-> byte helpers the vendored (GPL-2.0-or-later)
 * classes call.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef DMR_RX_MMDVM_UTILS_SHIM_H
#define DMR_RX_MMDVM_UTILS_SHIM_H

#include "Defines.h"

class CUtils {
public:
	// Most significant bit first.
	static inline void byteToBitsBE(unsigned char byte, bool* bits)
	{
		for (unsigned int i = 0U; i < 8U; i++)
			bits[i] = ((byte >> (7U - i)) & 0x01U) != 0U;
	}

	static inline void bitsToByteBE(const bool* bits, unsigned char& byte)
	{
		unsigned char b = 0U;
		for (unsigned int i = 0U; i < 8U; i++)
			b = (unsigned char)((b << 1) | (bits[i] ? 0x01U : 0x00U));
		byte = b;
	}
};

#endif
