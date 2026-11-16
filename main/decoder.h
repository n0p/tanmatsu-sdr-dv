// SPDX-License-Identifier: AGPL-3.0-only
// Glue between the DSP chain and the digital-voice protocol decoders (DMR, TETRA).
#pragma once

#include "app.h"

#define DEC_LINES    5
#define DEC_LINE_LEN 96
#define DEC_LOG_LINES 16  // decoder event history kept for the console

void dec_init(void);
// DSP task: called on retune and on mode change so the decoders drop their sync state.
void dec_reset(demod_mode_t mode);
// DSP task: 48 kHz FM discriminator samples (radians per sample).
void dec_dmr_feed(const float* fm, int n);
// DSP task: 72 kHz complex baseband (4 samples per symbol), channel centred at 0 Hz.
// The symbols the TETRA detector decides on (see tetra_rx_get_constellation), for the scope.
// Call from the task that feeds the decoder. Returns the number of symbols written to iq.
int  dec_tetra_constellation(cf32_t* iq, int max_symbols);
void dec_tetra_feed(const cf32_t* iq, int n);
// UI / console task: human readable decoder state for the info panel.
void dec_get_lines(demod_mode_t mode, char lines[DEC_LINES][DEC_LINE_LEN]);
// Any task: the timeslot whose speech is going to the speaker right now, 0 if none.
int dec_speaker_slot(void);
// Any task: the network (PLMN) of the TETRA cell the receiver is locked to. False if there is none.
bool dec_tetra_plmn(unsigned* mcc, unsigned* mnc);
// Any task: the most recent decoder events, newest first. Returns the number of lines copied.
int dec_get_log(char lines[][DEC_LINE_LEN], int max);
