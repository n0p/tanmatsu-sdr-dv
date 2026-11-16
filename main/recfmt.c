// SPDX-License-Identifier: AGPL-3.0-only
// Recording file formats, see recfmt.h.
#include "recfmt.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "opus.h"

#ifdef ESP_PLATFORM
#include "esp_heap_caps.h"
#include "esp_timer.h"
// The encoder state and the filter history are a few tens of KB: PSRAM, the internal heap is short.
static void* rf_alloc(size_t n) {
    return heap_caps_calloc(1, n, MALLOC_CAP_SPIRAM);
}
static void rf_free(void* p) {
    heap_caps_free(p);
}
static int64_t now_us(void) {
    return esp_timer_get_time();
}
#else
static void* rf_alloc(size_t n) {
    return calloc(1, n);
}
static void rf_free(void* p) {
    free(p);
}
static int64_t now_us(void) {
    return 0;
}
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ADPCM_BLOCK_BYTES   256
#define ADPCM_BLOCK_SAMPLES ((ADPCM_BLOCK_BYTES - 4) * 2 + 1)  // 505
#define OPUS_FRAME_MS       20
#define OPUS_PACKET_MAX     1275
// Encoder effort, 0..10. Measured on the ESP32-P4 per 20 ms frame, voice / wide setting:
// 0: 4.4 / 2.7 ms, 3: 5.7 ms / about 4, 5: 7.7 / 4.8 ms. The encoder shares a core with the display,
// so every millisecond here is display rate lost while recording (30 frames/s idle, 22 at 3, 19 at 5).
#define OPUS_COMPLEXITY     3

static const struct {
    const char* name;
    const char* ext;
    int         rate;     // sample rate the format works at
    int         frame;    // samples per encoder call at that rate
    int         bitrate;  // Opus only
    int         application;
} FORMATS[REC_FMT_COUNT] = {
    [REC_FMT_OPUS_VOICE] = {"Opus 16k", "opus", 16000, 16000 * OPUS_FRAME_MS / 1000, 16000, OPUS_APPLICATION_VOIP},
    [REC_FMT_OPUS_WIDE]  = {"Opus 32k", "opus", 24000, 24000 * OPUS_FRAME_MS / 1000, 32000, OPUS_APPLICATION_AUDIO},
    [REC_FMT_ADPCM]      = {"WAV ADPCM", "wav", 8000, ADPCM_BLOCK_SAMPLES, 0, 0},
};

const char* rec_format_name(rec_format_t fmt) {
    return (unsigned)fmt < REC_FMT_COUNT ? FORMATS[fmt].name : "?";
}

const char* rec_format_ext(rec_format_t fmt) {
    return (unsigned)fmt < REC_FMT_COUNT ? FORMATS[fmt].ext : "bin";
}

// ---- Decimating low-pass from 48 kHz ---------------------------------------------------------------

#define DECIM_MAX        6
#define DECIM_PER_PHASE  24  // taps per output sample: the transition band scales with the output rate
#define DECIM_TAPS_MAX   (DECIM_MAX * DECIM_PER_PHASE)
#define CHUNK            960  // input samples handled per call of the inner functions

typedef struct {
    int   d, ntaps;
    int   next;  // index, in the next block, of the newest input sample of the next output
    float h[DECIM_TAPS_MAX];
    float x[DECIM_TAPS_MAX - 1 + CHUNK];  // history followed by the new block
} decim_t;

static void decim_init(decim_t* q, int d) {
    q->d     = d;
    q->ntaps = d * DECIM_PER_PHASE;
    q->next  = 0;
    // Hamming-windowed sinc. Its transition band is 3.3 / ntaps wide (in cycles per input sample);
    // it is placed to end at the new Nyquist frequency, so nothing above that folds back.
    double trans = 3.3 / q->ntaps;
    double fc    = 0.5 / d - trans / 2;
    double sum   = 0;
    for (int k = 0; k < q->ntaps; k++) {
        double m  = k - (q->ntaps - 1) / 2.0;
        double s  = fabs(m) < 1e-9 ? 2 * fc : sin(2 * M_PI * fc * m) / (M_PI * m);
        double w  = 0.54 - 0.46 * cos(2 * M_PI * k / (q->ntaps - 1));
        q->h[k]   = (float)(s * w);
        sum      += s * w;
    }
    for (int k = 0; k < q->ntaps; k++) q->h[k] = (float)(q->h[k] / sum);
}

// n <= CHUNK. Returns the number of output samples (at most n / d + 1).
static int decim_run(decim_t* q, const int16_t* in, int n, int16_t* out) {
    float* fresh = q->x + q->ntaps - 1;
    for (int i = 0; i < n; i++) fresh[i] = (float)in[i];
    int no = 0, i = q->next;
    for (; i < n; i += q->d) {
        const float* x   = q->x + i;  // the ntaps samples ending at input sample i
        float        acc = 0.0f;
        for (int k = 0; k < q->ntaps; k++) acc += q->h[k] * x[k];
        if (acc > 32767.0f) acc = 32767.0f;
        if (acc < -32768.0f) acc = -32768.0f;
        out[no++] = (int16_t)lrintf(acc);
    }
    q->next = i - n;
    memmove(q->x, q->x + n, sizeof(float) * (size_t)(q->ntaps - 1));
    return no;
}

// ---- Ogg pages -------------------------------------------------------------------------------------

#define OGG_BODY_MAX     8192
#define OGG_PAGE_PACKETS 50  // one second of audio per page
#define OGG_FLAG_BOS     0x02
#define OGG_FLAG_EOS     0x04

typedef struct {
    FILE*    f;
    uint32_t serial, seq;
    uint64_t granule;  // 48 kHz sample position after the last packet on the page
    uint8_t  seg[255];
    int      nseg, body_len, packets;
    uint8_t  body[OGG_BODY_MAX];
} ogg_t;

static uint32_t ogg_crc_table[256];

static void ogg_crc_init(void) {
    if (ogg_crc_table[1]) return;
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t r = i << 24;
        for (int b = 0; b < 8; b++) r = (r & 0x80000000u) ? (r << 1) ^ 0x04c11db7u : r << 1;
        ogg_crc_table[i] = r;
    }
}

static uint32_t ogg_crc(uint32_t crc, const uint8_t* p, size_t n) {
    while (n--) crc = (crc << 8) ^ ogg_crc_table[((crc >> 24) ^ *p++) & 0xff];
    return crc;
}

static void put_le(uint8_t* p, uint64_t v, int bytes) {
    for (int i = 0; i < bytes; i++) p[i] = (uint8_t)(v >> (8 * i));
}

static bool ogg_flush(ogg_t* o, int flags) {
    if (o->nseg == 0) return true;
    uint8_t hdr[27 + 255];
    memcpy(hdr, "OggS", 4);
    hdr[4] = 0;
    hdr[5] = (uint8_t)flags;
    put_le(hdr + 6, o->granule, 8);
    put_le(hdr + 14, o->serial, 4);
    put_le(hdr + 18, o->seq, 4);
    put_le(hdr + 22, 0, 4);
    hdr[26] = (uint8_t)o->nseg;
    memcpy(hdr + 27, o->seg, (size_t)o->nseg);
    size_t   hlen = 27 + (size_t)o->nseg;
    uint32_t crc  = ogg_crc(ogg_crc(0, hdr, hlen), o->body, (size_t)o->body_len);
    put_le(hdr + 22, crc, 4);
    bool ok = fwrite(hdr, 1, hlen, o->f) == hlen && fwrite(o->body, 1, (size_t)o->body_len, o->f) == (size_t)o->body_len;
    o->seq++;
    o->nseg = o->body_len = o->packets = 0;
    return ok;
}

// Appends one packet (shorter than OGG_BODY_MAX), starting a new page first when this one is full,
// so the packet is always still in memory afterwards: the caller may yet mark its page as the last.
static bool ogg_packet(ogg_t* o, const uint8_t* p, int len, uint64_t granule_after) {
    bool ok   = true;
    int  need = len / 255 + 1;
    if (o->nseg + need > 255 || o->body_len + len > OGG_BODY_MAX || o->packets >= OGG_PAGE_PACKETS) ok = ogg_flush(o, 0);
    int rest = len;
    for (int i = 0; i < need - 1; i++, rest -= 255) o->seg[o->nseg++] = 255;
    o->seg[o->nseg++] = (uint8_t)rest;
    memcpy(o->body + o->body_len, p, (size_t)len);
    o->body_len += len;
    o->packets++;
    o->granule = granule_after;
    return ok;
}

// ---- IMA ADPCM -------------------------------------------------------------------------------------

static const int8_t  IMA_INDEX[8]  = {-1, -1, -1, -1, 2, 4, 6, 8};
static const int16_t IMA_STEP[89] = {
    7,    8,    9,    10,   11,   12,   13,   14,   16,   17,   19,   21,   23,    25,    28,    31,    34,    37,
    41,   45,   50,   55,   60,   66,   73,   80,   88,   97,   107,  118,  130,   143,   157,   173,   190,   209,
    230,  253,  279,  307,  337,  371,  408,  449,  494,  544,  598,  658,  724,   796,   876,   963,   1060,  1166,
    1282, 1411, 1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024, 3327, 3660, 4026,  4428,  4871,  5358,  5894,  6484,
    7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767,
};

typedef struct {
    int pred, index;
} ima_t;

static uint8_t ima_encode(ima_t* s, int16_t x) {
    int     step = IMA_STEP[s->index];
    int     diff = x - s->pred;
    uint8_t code = 0;
    if (diff < 0) {
        code = 8;
        diff = -diff;
    }
    int delta = step >> 3;  // what the decoder will add, built up with the same shifts it uses
    if (diff >= step) {
        code  |= 4;
        diff  -= step;
        delta += step;
    }
    if (diff >= step >> 1) {
        code  |= 2;
        diff  -= step >> 1;
        delta += step >> 1;
    }
    if (diff >= step >> 2) {
        code  |= 1;
        delta += step >> 2;
    }
    s->pred += (code & 8) ? -delta : delta;
    if (s->pred > 32767) s->pred = 32767;
    if (s->pred < -32768) s->pred = -32768;
    s->index += IMA_INDEX[code & 7];
    if (s->index < 0) s->index = 0;
    if (s->index > 88) s->index = 88;
    return code;
}

// ---- Writer ----------------------------------------------------------------------------------------

int recw_opus_complexity = OPUS_COMPLEXITY;

struct rec_writer {
    FILE*        f;
    rec_format_t fmt;
    bool         fail;
    int          rate, frame_len;
    uint64_t     in_samples;   // taken, at 48 kHz
    uint64_t     out_samples;  // after the decimator
    int16_t      frame[ADPCM_BLOCK_SAMPLES > 480 ? ADPCM_BLOCK_SAMPLES : 480];
    int          frame_n;
    uint32_t     enc_us_max;
    uint64_t     enc_us_sum;
    uint32_t     enc_calls;
    // Opus
    OpusEncoder* enc;
    int          lookahead;    // encoder delay, in samples at `rate`
    uint64_t     enc_samples;  // handed to the encoder, at `rate`
    uint8_t      pkt[OPUS_PACKET_MAX];
    // ADPCM
    ima_t        ima;
    uint32_t     blocks;
    // big members last
    decim_t      dec;
    ogg_t        ogg;
};

#define WAV_HEADER_BYTES 60

static bool wav_header(rec_writer_t* w, uint32_t samples) {
    uint8_t  h[WAV_HEADER_BYTES];
    uint32_t data = w->blocks * ADPCM_BLOCK_BYTES;
    memcpy(h, "RIFF", 4);
    put_le(h + 4, WAV_HEADER_BYTES - 8 + data, 4);
    memcpy(h + 8, "WAVEfmt ", 8);
    put_le(h + 16, 20, 4);
    put_le(h + 20, 0x0011, 2);  // WAVE_FORMAT_IMA_ADPCM
    put_le(h + 22, 1, 2);
    put_le(h + 24, (uint32_t)w->rate, 4);
    put_le(h + 28, (uint32_t)w->rate * ADPCM_BLOCK_BYTES / ADPCM_BLOCK_SAMPLES, 4);
    put_le(h + 32, ADPCM_BLOCK_BYTES, 2);
    put_le(h + 34, 4, 2);
    put_le(h + 36, 2, 2);
    put_le(h + 38, ADPCM_BLOCK_SAMPLES, 2);
    memcpy(h + 40, "fact", 4);
    put_le(h + 44, 4, 4);
    put_le(h + 48, samples, 4);
    memcpy(h + 52, "data", 4);
    put_le(h + 56, data, 4);
    return fwrite(h, 1, sizeof(h), w->f) == sizeof(h);
}

// One full block out of w->frame.
static bool adpcm_block(rec_writer_t* w) {
    uint8_t b[ADPCM_BLOCK_BYTES];
    w->ima.pred = w->frame[0];
    put_le(b, (uint16_t)w->frame[0], 2);
    b[2] = (uint8_t)w->ima.index;
    b[3] = 0;
    for (int i = 0; i < ADPCM_BLOCK_BYTES - 4; i++) {
        uint8_t lo = ima_encode(&w->ima, w->frame[1 + 2 * i]);
        uint8_t hi = ima_encode(&w->ima, w->frame[2 + 2 * i]);
        b[4 + i]   = (uint8_t)(lo | (hi << 4));
    }
    w->blocks++;
    return fwrite(b, 1, sizeof(b), w->f) == sizeof(b);
}

// One full frame out of w->frame.
static bool opus_frame(rec_writer_t* w) {
    int64_t t0  = now_us();
    int     len = opus_encode(w->enc, w->frame, w->frame_len, w->pkt, sizeof(w->pkt));
    int64_t dt  = now_us() - t0;
    if (dt > w->enc_us_max) w->enc_us_max = (uint32_t)dt;
    w->enc_us_sum += (uint64_t)dt;
    w->enc_calls++;
    if (len < 1) return false;
    w->enc_samples += (uint64_t)w->frame_len;
    return ogg_packet(&w->ogg, w->pkt, len, w->enc_samples * (uint64_t)(REC_IN_RATE / w->rate));
}

static bool frame_out(rec_writer_t* w) {
    w->frame_n = 0;
    return w->fmt == REC_FMT_ADPCM ? adpcm_block(w) : opus_frame(w);
}

static bool opus_start(rec_writer_t* w, const char* const* comments, int n_comments, uint32_t serial) {
    w->enc = rf_alloc((size_t)opus_encoder_get_size(1));
    if (!w->enc) return false;
    if (opus_encoder_init(w->enc, w->rate, 1, FORMATS[w->fmt].application) != OPUS_OK) return false;
    bool voice = w->fmt == REC_FMT_OPUS_VOICE;
    opus_encoder_ctl(w->enc, OPUS_SET_BITRATE(FORMATS[w->fmt].bitrate));
    opus_encoder_ctl(w->enc, OPUS_SET_VBR(1));
    opus_encoder_ctl(w->enc, OPUS_SET_COMPLEXITY(recw_opus_complexity));
    opus_encoder_ctl(w->enc, OPUS_SET_SIGNAL(voice ? OPUS_SIGNAL_VOICE : OPUS_AUTO));
    // No DTX: its speech detector takes steady tones and data bursts for a pause and replaces them
    // with comfort noise. Pauses are left out before they get here anyway (closed squelch, no call).
    opus_encoder_ctl(w->enc, OPUS_SET_LSB_DEPTH(16));
    opus_int32 la = 0;
    opus_encoder_ctl(w->enc, OPUS_GET_LOOKAHEAD(&la));
    w->lookahead = la;

    ogg_crc_init();
    w->ogg.f      = w->f;
    w->ogg.serial = serial;

    uint8_t head[19];
    memcpy(head, "OpusHead", 8);
    head[8] = 1;  // version
    head[9] = 1;  // channels
    put_le(head + 10, (uint32_t)la * (uint32_t)(REC_IN_RATE / w->rate), 2);  // pre-skip, in 48 kHz samples
    put_le(head + 12, (uint32_t)w->rate, 4);                                  // rate of the original
    put_le(head + 16, 0, 2);                                                  // output gain
    head[18] = 0;                                                             // one stream, mono
    if (!ogg_packet(&w->ogg, head, sizeof(head), 0) || !ogg_flush(&w->ogg, OGG_FLAG_BOS)) return false;

    static const char vendor[] = "tanmatsu-sdr, libopus 1.5.2";
    uint8_t           tags[512];
    int               n = 0;
    memcpy(tags, "OpusTags", 8);
    n = 8;
    put_le(tags + n, sizeof(vendor) - 1, 4);
    memcpy(tags + n + 4, vendor, sizeof(vendor) - 1);
    n         += 4 + (int)sizeof(vendor) - 1;
    int count  = n;  // the number of comments goes here once it is known
    n         += 4;
    int used   = 0;
    for (int i = 0; i < n_comments; i++) {
        size_t len = strlen(comments[i]);
        if ((size_t)n + 4 + len > sizeof(tags)) break;
        put_le(tags + n, len, 4);
        memcpy(tags + n + 4, comments[i], len);
        n += 4 + (int)len;
        used++;
    }
    put_le(tags + count, (uint32_t)used, 4);
    return ogg_packet(&w->ogg, tags, n, 0) && ogg_flush(&w->ogg, 0);
}

rec_writer_t* recw_open(FILE* f, rec_format_t fmt, const char* const* comments, int n_comments, uint32_t serial) {
    if ((unsigned)fmt >= REC_FMT_COUNT) return NULL;
    rec_writer_t* w = rf_alloc(sizeof(*w));
    if (!w) return NULL;
    w->f         = f;
    w->fmt       = fmt;
    w->rate      = FORMATS[fmt].rate;
    w->frame_len = FORMATS[fmt].frame;
    decim_init(&w->dec, REC_IN_RATE / w->rate);
    bool ok = fmt == REC_FMT_ADPCM ? wav_header(w, 0) : opus_start(w, comments, n_comments, serial);
    if (!ok) {
        rf_free(w->enc);
        rf_free(w);
        return NULL;
    }
    return w;
}

bool recw_write(rec_writer_t* w, const int16_t* pcm, int n) {
    int16_t low[CHUNK / 2 + 1];
    while (n > 0 && !w->fail) {
        int take = n < CHUNK ? n : CHUNK;
        int m    = decim_run(&w->dec, pcm, take, low);
        pcm           += take;
        n             -= take;
        w->in_samples += (uint64_t)take;
        w->out_samples += (uint64_t)m;
        for (int i = 0; i < m; i++) {
            w->frame[w->frame_n++] = low[i];
            if (w->frame_n == w->frame_len && !frame_out(w)) w->fail = true;
        }
    }
    return !w->fail;
}

bool recw_checkpoint(rec_writer_t* w) {
    if (w->fail) return false;
    if (w->fmt == REC_FMT_ADPCM) {
        // Rewrite the header for the blocks written so far, then go back to the end.
        if (fseek(w->f, 0, SEEK_SET) != 0 || !wav_header(w, w->blocks * ADPCM_BLOCK_SAMPLES) || fseek(w->f, 0, SEEK_END) != 0)
            w->fail = true;
    } else if (!ogg_flush(&w->ogg, 0)) {
        w->fail = true;
    }
    if (fflush(w->f) != 0) w->fail = true;
    return !w->fail;
}

bool recw_close(rec_writer_t* w) {
    bool ok = !w->fail;
    if (w->fmt == REC_FMT_ADPCM) {
        if (w->frame_n > 0) {  // fill the last block by holding the last sample; `fact` has the true length
            int16_t last = w->frame[w->frame_n - 1];
            while (w->frame_n < w->frame_len) w->frame[w->frame_n++] = last;
            ok = ok && frame_out(w);
        }
        ok = ok && fseek(w->f, 0, SEEK_SET) == 0 && wav_header(w, (uint32_t)w->out_samples) && fseek(w->f, 0, SEEK_END) == 0;
    } else {
        // The decoder drops the first `lookahead` samples it produces (the pre-skip), so that many
        // more have to go in at the end to get the last of the audio out. The granule position of
        // the last page then cuts the padding off again.
        uint64_t need = w->out_samples + (uint64_t)w->lookahead;
        do {
            memset(w->frame + w->frame_n, 0, sizeof(int16_t) * (size_t)(w->frame_len - w->frame_n));
            ok = ok && frame_out(w);
        } while (ok && w->enc_samples < need);
        w->ogg.granule = need * (uint64_t)(REC_IN_RATE / w->rate);
        ok             = ok && ogg_flush(&w->ogg, OGG_FLAG_EOS);
    }
    ok = fflush(w->f) == 0 && ok;
    rf_free(w->enc);
    rf_free(w);
    return ok;
}

uint32_t recw_ms(const rec_writer_t* w) {
    return (uint32_t)(w->in_samples * 1000 / REC_IN_RATE);
}

uint32_t recw_enc_us_avg(const rec_writer_t* w) {
    return w->enc_calls ? (uint32_t)(w->enc_us_sum / w->enc_calls) : 0;
}

uint32_t recw_enc_us_max(rec_writer_t* w) {
    uint32_t v    = w->enc_us_max;
    w->enc_us_max = 0;
    return v;
}
