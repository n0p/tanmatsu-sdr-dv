/*
 * Minimal stand-in for <osmocom/core/utils.h> so that the vendored osmo-tetra
 * sources build without libosmocore (tanmatsu-sdr tetra_rx component).
 *
 * Only the handful of helpers the vendored files actually use are provided.
 * This file is NOT copied from libosmocore; it re-implements the trivial
 * value_string lookup with the same interface (no static scratch buffer, so
 * unknown values map to a constant string instead of "unknown 0x..").
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#ifndef TETRA_RX_OSMO_COMPAT_UTILS_H
#define TETRA_RX_OSMO_COMPAT_UTILS_H

#include <stddef.h>
#include <stdint.h>

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

struct value_string {
	uint32_t value;
	const char *str;
};

static inline const char *get_value_string(const struct value_string *vs, uint32_t val)
{
	for (; vs->str != NULL; vs++) {
		if (vs->value == val)
			return vs->str;
	}
	return "unknown";
}

#endif
