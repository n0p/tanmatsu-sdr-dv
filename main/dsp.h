// SPDX-License-Identifier: AGPL-3.0-only
// DSP primitives for the Tanmatsu SDR receiver: filter design, decimators, FFT, demod helpers.
// Everything is single-precision float or integer: the ESP32-P4 has no double FPU.
#pragma once

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float re, im;
} cf32_t;

// Windowed-sinc lowpass with a Kaiser window. fc is the -6 dB cutoff as a fraction of the sample
// rate (0..0.5). atten_db sets the stopband attenuation. Taps are normalised to unity DC gain.
void dsp_design_lowpass(float* h, int ntaps, float fc, float atten_db);

// ---- Stage 1: CU8 IQ -> cf32, combined fs/4 downshift + integer FIR + decimation -----------------
// The wanted channel sits at +fs/4 in the raw stream (the tuner LO is parked fs/4 below it so the
// tuner's DC spur and 1/f noise stay out of the channel). Decimation must be a multiple of 4, which
// makes the fs/4 rotation a fixed per-tap sign/swap pattern that is folded into the coefficients.
#define STAGE1_MAX_TAPS 64

typedef struct {
    int16_t cI[STAGE1_MAX_TAPS];
    int16_t cQ[STAGE1_MAX_TAPS];
    int     ntaps;  // multiple of 4
    int     decim;  // multiple of 4
    float   kI, kQ;  // DC term of the unsigned input, in accumulator units
    float   scale;   // accumulator -> +/-1.0 full scale
} stage1_t;

void stage1_init(stage1_t* s, int ntaps, int decim, float fc, float atten_db);
// Consumes whole decimation steps from raw (nsamp complex samples). Returns outputs written;
// *consumed is the number of input samples that can be dropped (the rest must be kept as history).
int stage1_run(const stage1_t* s, const uint8_t* raw, int nsamp, cf32_t* out, int* consumed);

// ---- Complex float FIR decimator -----------------------------------------------------------------
typedef struct {
    float*  taps;
    int     ntaps;
    int     decim;
    cf32_t* buf;  // history + pending input
    int     fill;
    int     cap;
} cfir_t;

bool cfir_init(cfir_t* f, int ntaps, int decim, int max_in);
void cfir_design(cfir_t* f, float fc, float atten_db);  // also clears history
void cfir_reset(cfir_t* f);
int  cfir_run(cfir_t* f, const cf32_t* in, int n, cf32_t* out);

// ---- Real float FIR decimator --------------------------------------------------------------------
typedef struct {
    float* taps;
    int    ntaps;
    int    decim;
    float* buf;
    int    fill;
    int    cap;
} rfir_t;

bool rfir_init(rfir_t* f, int ntaps, int decim, int max_in);
void rfir_design(rfir_t* f, float fc, float atten_db);
void rfir_reset(rfir_t* f);
int  rfir_run(rfir_t* f, const float* in, int n, float* out);

// ---- Biquad (direct form II transposed) ----------------------------------------------------------
typedef struct {
    float b0, b1, b2, a1, a2;
    float z1, z2;
} biquad_t;

void biquad_lowpass(biquad_t* q, float fc, float fs, float Q);
void biquad_highpass(biquad_t* q, float fc, float fs, float Q);
static inline float biquad_run(biquad_t* q, float x) {
    float y = q->b0 * x + q->z1;
    q->z1   = q->b1 * x - q->a1 * y + q->z2;
    q->z2   = q->b2 * x - q->a2 * y;
    return y;
}

// ---- FFT -----------------------------------------------------------------------------------------
#define DSP_FFT_MAX 2048
void dsp_fft_init(void);
// In-place forward FFT, n must be a power of two <= DSP_FFT_MAX.
void dsp_fft(cf32_t* x, int n);
// Hann window of length DSP_FFT_MAX (index with a stride for shorter transforms).
extern float dsp_hann[DSP_FFT_MAX];

// ---- Misc ----------------------------------------------------------------------------------------
// atan2 approximation, max error about 1e-4 rad.
static inline float dsp_atan2f(float y, float x) {
    float ax = fabsf(x), ay = fabsf(y);
    float mx = ax > ay ? ax : ay;
    float mn = ax > ay ? ay : ax;
    if (mx <= 0.0f) return 0.0f;
    float a = mn / mx;
    float s = a * a;
    float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
    if (ay > ax) r = 1.57079637f - r;
    if (x < 0.0f) r = 3.14159274f - r;
    return y < 0.0f ? -r : r;
}

// FM discriminator: phase difference between consecutive samples, in radians (-pi..pi).
typedef struct {
    cf32_t prev;
} fmdisc_t;

static inline float fmdisc_run(fmdisc_t* d, cf32_t x) {
    float re = x.re * d->prev.re + x.im * d->prev.im;
    float im = x.im * d->prev.re - x.re * d->prev.im;
    d->prev  = x;
    return dsp_atan2f(im, re);
}

// ---- SSB demodulator -----------------------------------------------------------------------------
// Input: 48 kHz complex baseband with the (suppressed) carrier at offset_hz. The wanted sideband is
// shifted so that its centre, 1.5 kHz from the carrier, lands on 0 Hz, decimated to 12 kHz, cut out
// with a +-1.4 kHz low-pass and shifted back to audio. Audio band 0.3 .. 2.7 kHz; the other
// sideband is more than 60 dB down from 70 Hz beyond the carrier. Output: 48 kHz audio.
#define SSB_IN_RATE  48000
#define SSB_UP       4
#define SSB_UP_TAPS  32
#define SSB_PITCH_HZ 1500.0f

typedef struct {
    cfir_t dec;         // 48 kHz -> 12 kHz
    cfir_t chan;        // sideband filter at 12 kHz
    cf32_t osc, step;   // tuning oscillator at 48 kHz
    int    sideband;    // +1 upper, -1 lower
    int    ph;          // phase of the 1.5 kHz (fs/8) shift back to audio
    bool   agc;         // false: unity gain (tests)
    float  env;         // AGC envelope
    int    hang;        // samples left before the envelope may decay
    int    settle;      // samples left of the muted start, while filters and envelope fill up
    float  level_db;    // power in the sideband filter over the last block, dBFS
    float  up_h[SSB_UP_TAPS];
    float  up_x[SSB_UP_TAPS / SSB_UP];
} ssb_t;

bool ssb_init(ssb_t* s, int max_in);
void ssb_reset(ssb_t* s);
// sideband: +1 upper, -1 lower. offset_hz: where the carrier sits in the input.
void ssb_tune(ssb_t* s, int sideband, float offset_hz);
// io: n input samples, overwritten. Returns the number of audio samples written to out (about n).
int ssb_run(ssb_t* s, cf32_t* io, int n, float* out);

#ifdef __cplusplus
}
#endif
