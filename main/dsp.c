// SPDX-License-Identifier: AGPL-3.0-only
#include "dsp.h"
#include <stdlib.h>
#include <string.h>

#ifdef ESP_PLATFORM
#include "esp_attr.h"
#include "esp_heap_caps.h"
#define DSP_HOT         IRAM_ATTR
#define dsp_alloc(size) heap_caps_malloc((size), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
#else
#define DSP_HOT
#define dsp_alloc(size) malloc(size)
#endif

#define PI_F 3.14159265358979f

// Modified Bessel function of the first kind, order 0 (series).
static float bessel_i0(float x) {
    float sum = 1.0f, term = 1.0f;
    float q = x * x * 0.25f;
    for (int k = 1; k < 40; k++) {
        term *= q / ((float)k * (float)k);
        sum  += term;
        if (term < sum * 1e-9f) break;
    }
    return sum;
}

void dsp_design_lowpass(float* h, int ntaps, float fc, float atten_db) {
    float beta;
    if (atten_db > 50.0f) {
        beta = 0.1102f * (atten_db - 8.7f);
    } else if (atten_db > 21.0f) {
        beta = 0.5842f * powf(atten_db - 21.0f, 0.4f) + 0.07886f * (atten_db - 21.0f);
    } else {
        beta = 0.0f;
    }
    float i0b = bessel_i0(beta);
    float mid = (float)(ntaps - 1) * 0.5f;
    float sum = 0.0f;
    for (int i = 0; i < ntaps; i++) {
        float t    = (float)i - mid;
        float sinc = (fabsf(t) < 1e-6f) ? 2.0f * fc : sinf(2.0f * PI_F * fc * t) / (PI_F * t);
        float r    = mid > 0.0f ? t / mid : 0.0f;
        float w    = bessel_i0(beta * sqrtf(fmaxf(0.0f, 1.0f - r * r))) / i0b;
        h[i]       = sinc * w;
        sum       += h[i];
    }
    for (int i = 0; i < ntaps; i++) h[i] /= sum;
}

// ---- Stage 1 -------------------------------------------------------------------------------------

void stage1_init(stage1_t* s, int ntaps, int decim, float fc, float atten_db) {
    float h[STAGE1_MAX_TAPS];
    if (ntaps > STAGE1_MAX_TAPS) ntaps = STAGE1_MAX_TAPS;
    ntaps &= ~3;
    dsp_design_lowpass(h, ntaps, fc, atten_db);
    // Tap j multiplies x[n+j] * exp(-j*pi/2*j):
    //   j%4==0: ( I,  Q)   j%4==1: ( Q, -I)   j%4==2: (-I, -Q)   j%4==3: (-Q,  I)
    static const int8_t sign_i[4] = {1, 1, -1, -1};
    static const int8_t sign_q[4] = {1, -1, -1, 1};
    int32_t sum_i = 0, sum_q = 0;
    for (int j = 0; j < ntaps; j++) {
        int c    = (int)lrintf(h[j] * 32768.0f);
        s->cI[j] = (int16_t)(c * sign_i[j & 3]);
        s->cQ[j] = (int16_t)(c * sign_q[j & 3]);
        sum_i   += s->cI[j];
        sum_q   += s->cQ[j];
    }
    s->ntaps = ntaps;
    s->decim = decim;
    s->kI    = 127.5f * (float)sum_i;
    s->kQ    = 127.5f * (float)sum_q;
    s->scale = 1.0f / (127.5f * 32768.0f);
}

int DSP_HOT stage1_run(const stage1_t* s, const uint8_t* raw, int nsamp, cf32_t* out, int* consumed) {
    const int      nt = s->ntaps;
    const int      D  = s->decim;
    const int16_t* cI = s->cI;
    const int16_t* cQ = s->cQ;
    int            n  = 0;
    int            pos = 0;
    while (pos + nt <= nsamp) {
        const uint8_t* p  = raw + 2 * pos;
        int32_t        aI = 0, aQ = 0;
        for (int j = 0; j < nt; j += 4, p += 8) {
            aI += cI[j] * p[0] + cI[j + 1] * p[3] + cI[j + 2] * p[4] + cI[j + 3] * p[7];
            aQ += cQ[j] * p[1] + cQ[j + 1] * p[2] + cQ[j + 2] * p[5] + cQ[j + 3] * p[6];
        }
        out[n].re = ((float)aI - s->kI) * s->scale;
        out[n].im = ((float)aQ - s->kQ) * s->scale;
        n++;
        pos += D;
    }
    *consumed = pos;
    return n;
}

// ---- Complex FIR ---------------------------------------------------------------------------------

bool cfir_init(cfir_t* f, int ntaps, int decim, int max_in) {
    memset(f, 0, sizeof(*f));
    f->ntaps = ntaps;
    f->decim = decim;
    f->cap   = ntaps + decim + max_in;
    f->taps  = dsp_alloc(sizeof(float) * (size_t)ntaps);
    f->buf   = dsp_alloc(sizeof(cf32_t) * (size_t)f->cap);
    if (!f->taps || !f->buf) return false;
    memset(f->taps, 0, sizeof(float) * (size_t)ntaps);
    cfir_reset(f);
    return true;
}

void cfir_reset(cfir_t* f) {
    // Prime with ntaps-1 zeros so the first output is aligned with the first input.
    f->fill = f->ntaps - 1;
    memset(f->buf, 0, sizeof(cf32_t) * (size_t)f->fill);
}

void cfir_design(cfir_t* f, float fc, float atten_db) {
    dsp_design_lowpass(f->taps, f->ntaps, fc, atten_db);
    cfir_reset(f);
}

int DSP_HOT cfir_run(cfir_t* f, const cf32_t* in, int n, cf32_t* out) {
    if (n > f->cap - f->fill) n = f->cap - f->fill;
    memcpy(f->buf + f->fill, in, sizeof(cf32_t) * (size_t)n);
    f->fill += n;
    const int    nt = f->ntaps;
    const float* h  = f->taps;
    int          no = 0, pos = 0;
    while (pos + nt <= f->fill) {
        const cf32_t* p  = f->buf + pos;
        float         ar = 0.0f, ai = 0.0f, br = 0.0f, bi = 0.0f;
        int           j  = 0;
        for (; j + 1 < nt; j += 2) {
            ar += h[j] * p[j].re;
            ai += h[j] * p[j].im;
            br += h[j + 1] * p[j + 1].re;
            bi += h[j + 1] * p[j + 1].im;
        }
        if (j < nt) {
            ar += h[j] * p[j].re;
            ai += h[j] * p[j].im;
        }
        out[no].re = ar + br;
        out[no].im = ai + bi;
        no++;
        pos += f->decim;
    }
    f->fill -= pos;
    memmove(f->buf, f->buf + pos, sizeof(cf32_t) * (size_t)f->fill);
    return no;
}

// ---- Real FIR ------------------------------------------------------------------------------------

bool rfir_init(rfir_t* f, int ntaps, int decim, int max_in) {
    memset(f, 0, sizeof(*f));
    f->ntaps = ntaps;
    f->decim = decim;
    f->cap   = ntaps + decim + max_in;
    f->taps  = dsp_alloc(sizeof(float) * (size_t)ntaps);
    f->buf   = dsp_alloc(sizeof(float) * (size_t)f->cap);
    if (!f->taps || !f->buf) return false;
    memset(f->taps, 0, sizeof(float) * (size_t)ntaps);
    rfir_reset(f);
    return true;
}

void rfir_reset(rfir_t* f) {
    f->fill = f->ntaps - 1;
    memset(f->buf, 0, sizeof(float) * (size_t)f->fill);
}

void rfir_design(rfir_t* f, float fc, float atten_db) {
    dsp_design_lowpass(f->taps, f->ntaps, fc, atten_db);
    rfir_reset(f);
}

int DSP_HOT rfir_run(rfir_t* f, const float* in, int n, float* out) {
    if (n > f->cap - f->fill) n = f->cap - f->fill;
    memcpy(f->buf + f->fill, in, sizeof(float) * (size_t)n);
    f->fill += n;
    const int    nt = f->ntaps;
    const float* h  = f->taps;
    int          no = 0, pos = 0;
    while (pos + nt <= f->fill) {
        const float* p = f->buf + pos;
        float        a = 0.0f, b = 0.0f;
        int          j = 0;
        for (; j + 1 < nt; j += 2) {
            a += h[j] * p[j];
            b += h[j + 1] * p[j + 1];
        }
        if (j < nt) a += h[j] * p[j];
        out[no++] = a + b;
        pos      += f->decim;
    }
    f->fill -= pos;
    memmove(f->buf, f->buf + pos, sizeof(float) * (size_t)f->fill);
    return no;
}

// ---- Biquads (RBJ cookbook) ----------------------------------------------------------------------

void biquad_lowpass(biquad_t* q, float fc, float fs, float Q) {
    float w0 = 2.0f * PI_F * fc / fs;
    float cs = cosf(w0), al = sinf(w0) / (2.0f * Q);
    float a0 = 1.0f + al;
    q->b0    = (1.0f - cs) * 0.5f / a0;
    q->b1    = (1.0f - cs) / a0;
    q->b2    = q->b0;
    q->a1    = -2.0f * cs / a0;
    q->a2    = (1.0f - al) / a0;
    q->z1 = q->z2 = 0.0f;
}

void biquad_highpass(biquad_t* q, float fc, float fs, float Q) {
    float w0 = 2.0f * PI_F * fc / fs;
    float cs = cosf(w0), al = sinf(w0) / (2.0f * Q);
    float a0 = 1.0f + al;
    q->b0    = (1.0f + cs) * 0.5f / a0;
    q->b1    = -(1.0f + cs) / a0;
    q->b2    = q->b0;
    q->a1    = -2.0f * cs / a0;
    q->a2    = (1.0f - al) / a0;
    q->z1 = q->z2 = 0.0f;
}

// ---- SSB -----------------------------------------------------------------------------------------

#define SSB_AGC_TARGET   0.35f     // audio peak the AGC steers to
#define SSB_AGC_MAX_GAIN 10000.0f  // 80 dB
#define SSB_AGC_ATTACK   0.2f      // per 12 kHz sample
#define SSB_AGC_HANG     3000      // 0.25 s
#define SSB_AGC_DECAY    0.99985f  // then about 0.55 s time constant

bool ssb_init(ssb_t* s, int max_in) {
    memset(s, 0, sizeof(*s));
    if (!cfir_init(&s->dec, 24, 4, max_in)) return false;
    if (!cfir_init(&s->chan, 128, 1, max_in / 4 + 4)) return false;
    // Decimator: only what would alias into the sideband filter has to go (beyond 10.4 kHz).
    cfir_design(&s->dec, 6000.0f / (float)SSB_IN_RATE, 60.0f);
    cfir_design(&s->chan, 1400.0f / (float)(SSB_IN_RATE / 4), 60.0f);
    // Interpolator back to 48 kHz: audio ends at 2.9 kHz, its first image starts at 9.1 kHz.
    dsp_design_lowpass(s->up_h, SSB_UP_TAPS, 6000.0f / (float)SSB_IN_RATE, 60.0f);
    for (int i = 0; i < SSB_UP_TAPS; i++) s->up_h[i] *= (float)SSB_UP;
    s->agc = true;
    ssb_tune(s, 1, 0.0f);
    ssb_reset(s);
    return true;
}

void ssb_reset(ssb_t* s) {
    cfir_reset(&s->dec);
    cfir_reset(&s->chan);
    s->osc.re   = 1.0f;
    s->osc.im   = 0.0f;
    s->ph       = 0;
    s->env      = 0.0f;
    s->hang     = 0;
    s->settle   = s->chan.ntaps + 32;
    s->level_db = -200.0f;
    memset(s->up_x, 0, sizeof(s->up_x));
}

void ssb_tune(ssb_t* s, int sideband, float offset_hz) {
    s->sideband = sideband < 0 ? -1 : 1;
    float w     = -2.0f * PI_F * (offset_hz + (float)s->sideband * SSB_PITCH_HZ) / (float)SSB_IN_RATE;
    s->step.re  = cosf(w);
    s->step.im  = sinf(w);
}

int DSP_HOT ssb_run(ssb_t* s, cf32_t* io, int n, float* out) {
    // cos(2 pi k / 8); the sine is the same table a quarter turn back.
    static const float c8[8] = {1.0f, 0.70710678f, 0.0f, -0.70710678f, -1.0f, -0.70710678f, 0.0f, 0.70710678f};

    cf32_t o = s->osc, st = s->step;
    for (int i = 0; i < n; i++) {
        float re = io[i].re * o.re - io[i].im * o.im;
        float im = io[i].re * o.im + io[i].im * o.re;
        io[i].re = re;
        io[i].im = im;
        float t  = o.re * st.re - o.im * st.im;
        o.im     = o.re * st.im + o.im * st.re;
        o.re     = t;
    }
    // Keep the recursive oscillator on the unit circle.
    float corr = 1.5f - 0.5f * (o.re * o.re + o.im * o.im);
    s->osc.re  = o.re * corr;
    s->osc.im  = o.im * corr;

    n = cfir_run(&s->dec, io, n, io);
    n = cfir_run(&s->chan, io, n, io);
    if (n <= 0) return 0;

    const float sb    = (float)s->sideband;
    const int   nx    = SSB_UP_TAPS / SSB_UP;
    float       power = 0.0f;
    for (int i = 0; i < n; i++) {
        float p2  = io[i].re * io[i].re + io[i].im * io[i].im;
        power    += p2;
        float gain = 1.0f;
        if (s->agc) {
            float mag = sqrtf(p2);
            if (s->settle > 0) {
                // After a reset the envelope is unknown: follow the signal at once and stay silent,
                // instead of starting at full gain.
                s->settle--;
                if (mag > s->env) s->env = mag;
                s->hang = SSB_AGC_HANG / 4;
                mag     = 0.0f;
                io[i].re = io[i].im = 0.0f;
            } else if (mag > s->env) {
                s->env += SSB_AGC_ATTACK * (mag - s->env);
                s->hang = SSB_AGC_HANG;
            } else if (s->hang > 0) {
                s->hang--;
            } else {
                s->env *= SSB_AGC_DECAY;
            }
            gain = SSB_AGC_TARGET / (s->env + SSB_AGC_TARGET / SSB_AGC_MAX_GAIN);
        }
        // Real part of z * exp(+-j 2 pi 1500 t): the sideband centre goes back to 1.5 kHz audio.
        float a = (io[i].re * c8[s->ph] - sb * io[i].im * c8[(s->ph + 6) & 7]) * gain;
        s->ph   = (s->ph + 1) & 7;

        memmove(s->up_x + 1, s->up_x, sizeof(float) * (size_t)(nx - 1));
        s->up_x[0] = a;
        for (int p = 0; p < SSB_UP; p++) {
            float acc = 0.0f;
            for (int k = 0; k < nx; k++) acc += s->up_h[p + SSB_UP * k] * s->up_x[k];
            out[i * SSB_UP + p] = acc;
        }
    }
    s->level_db = 10.0f * log10f(power / (float)n + 1e-20f);
    return n * SSB_UP;
}

// ---- FFT -----------------------------------------------------------------------------------------

float         dsp_hann[DSP_FFT_MAX];
static cf32_t fft_tw[DSP_FFT_MAX / 2];
static bool   fft_ready = false;

void dsp_fft_init(void) {
    if (fft_ready) return;
    for (int i = 0; i < DSP_FFT_MAX / 2; i++) {
        float a      = -2.0f * PI_F * (float)i / (float)DSP_FFT_MAX;
        fft_tw[i].re = cosf(a);
        fft_tw[i].im = sinf(a);
    }
    for (int i = 0; i < DSP_FFT_MAX; i++) {
        dsp_hann[i] = 0.5f - 0.5f * cosf(2.0f * PI_F * (float)i / (float)DSP_FFT_MAX);
    }
    fft_ready = true;
}

void dsp_fft(cf32_t* x, int n) {
    // Bit reversal
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            cf32_t t = x[i];
            x[i]     = x[j];
            x[j]     = t;
        }
    }
    for (int len = 2; len <= n; len <<= 1) {
        int half = len >> 1;
        int step = DSP_FFT_MAX / len;
        for (int i = 0; i < n; i += len) {
            cf32_t* a = x + i;
            cf32_t* b = a + half;
            for (int k = 0; k < half; k++) {
                cf32_t w  = fft_tw[k * step];
                float  tr = b[k].re * w.re - b[k].im * w.im;
                float  ti = b[k].re * w.im + b[k].im * w.re;
                b[k].re   = a[k].re - tr;
                b[k].im   = a[k].im - ti;
                a[k].re  += tr;
                a[k].im  += ti;
            }
        }
    }
}
