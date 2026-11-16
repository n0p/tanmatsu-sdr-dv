// SPDX-License-Identifier: AGPL-3.0-only
// Recording file formats: 48 kHz mono audio in, an Ogg Opus or IMA ADPCM WAV file out.
// Plain C on a FILE*, no ESP-IDF calls, so tests/test_rec.c runs it on the PC.
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define REC_IN_RATE 48000

typedef enum {
    REC_FMT_OPUS_VOICE = 0,  // Ogg Opus, 16 kHz, 16 kbit/s: about 7 MB per hour
    REC_FMT_OPUS_WIDE,       // Ogg Opus, 24 kHz, 32 kbit/s: about 14 MB per hour (broadcast FM)
    REC_FMT_ADPCM,           // WAV, IMA ADPCM, 8 kHz, 4 bit: about 15 MB per hour, plays anywhere
    REC_FMT_COUNT,
} rec_format_t;

const char* rec_format_name(rec_format_t fmt);  // for the settings menu
const char* rec_format_ext(rec_format_t fmt);   // "opus" or "wav"

typedef struct rec_writer rec_writer_t;

// Writes the file header. `comments` are "KEY=value" tags for the Opus file (ignored for WAV),
// `serial` identifies the Ogg stream and should differ from file to file. NULL when out of memory
// or when the header could not be written.
rec_writer_t* recw_open(FILE* f, rec_format_t fmt, const char* const* comments, int n_comments, uint32_t serial);

// Audio at REC_IN_RATE. False once writing to the file has failed.
bool recw_write(rec_writer_t* w, const int16_t* pcm, int n);

// Brings the file on disk to a state a player accepts if the power went away right now (the WAV
// header carries the length; Ogg pages are complete by themselves). The caller still has to fsync.
bool recw_checkpoint(rec_writer_t* w);

// Finishes the file (last frame, end-of-stream page, final lengths) and frees the writer. The
// FILE stays open and belongs to the caller.
bool recw_close(rec_writer_t* w);

uint32_t recw_ms(const rec_writer_t* w);       // audio taken so far
uint32_t recw_enc_us_max(rec_writer_t* w);     // longest single encoder call since the last query (0 on the PC)
uint32_t recw_enc_us_avg(const rec_writer_t* w);  // mean encoder time per 20 ms frame (0 on the PC)

// Opus encoder effort, 0..10, for files opened from now on.
extern int recw_opus_complexity;
