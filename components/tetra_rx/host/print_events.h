/* text formatting of tetra_rx events (CLI and tests)
 * SPDX-License-Identifier: AGPL-3.0-or-later */
#ifndef PRINT_EVENTS_H
#define PRINT_EVENTS_H

#include <stdio.h>
#include "tetra_rx.h"

/* one line (no newline) describing the event; returns the length */
int format_event(char *buf, size_t size, const tetra_rx_event_t *ev);
void print_status(FILE *f, const tetra_rx_status_t *st);
/* FNV-1a over a string */
uint64_t fnv1a(uint64_t h, const char *s);

#endif
