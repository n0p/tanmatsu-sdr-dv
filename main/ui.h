// SPDX-License-Identifier: AGPL-3.0-only
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Brings up the display (after bsp_device_initialize) and allocates the UI buffers.
bool ui_init(void);
// Renders one frame and presents it.
void ui_frame(void);

// The text of the bottom line, and whether it is a prompt or notice rather than the key help.
void ui_help_text(char* text, size_t len, bool* highlight);
// Power in the demodulated band over the noise floor beside it, dB, from the spectrum on screen.
// False until there is a spectrum.
bool ui_snr_db(float* snr);

// Frequency entry state, owned by main.c.
extern char          ui_entry[20];
extern volatile bool ui_entry_active;

extern const uint32_t step_table[];
extern const int      step_count;
