/*
 * dmr_rx - 4FSK front end: matched filter, sync correlators, burst slicer.
 *
 * Signal path: 48 kS/s discriminator samples -> 81 tap root raised cosine
 * (alpha 0.2) evaluated at every second input sample -> 24 kS/s ring buffer
 * (5 samples per symbol). Everything after that works on the ring:
 *
 *  - sync search: for every new sample the 24 symbol-spaced samples ending
 *    there are correlated against the four sync families. All sync symbols
 *    are +-3 and every pattern is DC balanced, so correlation, mean and
 *    energy give a closed-form least-squares fit (level, offset, residual)
 *    that is independent of gain, DC offset and polarity.
 *  - burst slicing: the 144 symbols (CACH + burst) are interpolated (cubic
 *    Lagrange) at the sync derived instant, the timing is refined by
 *    minimising the slicer error energy and the levels by a decision
 *    directed least-squares refit.
 *
 * SPDX-License-Identifier: AGPL-3.0-only
 */
#include <math.h>
#include <string.h>

#include "dmr_internal.h"

namespace dmrrx {

const uint8_t kVoiceSync[kNumFamilies][6] = {
    {0x75, 0x5F, 0xD7, 0xDF, 0x75, 0xF7}, /* BS sourced voice     */
    {0x7F, 0x7D, 0x5D, 0xD5, 0x7D, 0xFD}, /* MS sourced voice     */
    {0x5D, 0x57, 0x7F, 0x77, 0x57, 0xFF}, /* direct mode TS1 voice */
    {0x7D, 0xFF, 0xD5, 0xF5, 0x5D, 0x5F}, /* direct mode TS2 voice */
};

static const float kRho2Low = 0.60f; /* candidate threshold */

void Demod::init()
{
    /* Root raised cosine, alpha = 0.2, 10 samples per symbol, +-4 symbols,
     * unity DC gain. Only the first half is stored (symmetric). */
    const float alpha = 0.2f;
    const float pi    = 3.14159265358979f;
    float h[kFirTaps];
    float sum = 0.0f;
    for (int k = 0; k < kFirTaps; k++) {
        const float t = (float)(k - kFirTaps / 2) / 10.0f;
        float v;
        const float den = 1.0f - (4.0f * alpha * t) * (4.0f * alpha * t);
        if (k == kFirTaps / 2) {
            v = 1.0f - alpha + 4.0f * alpha / pi;
        } else if (fabsf(den) < 1e-4f) {
            v = alpha / sqrtf(2.0f) *
                ((1.0f + 2.0f / pi) * sinf(pi / (4.0f * alpha)) + (1.0f - 2.0f / pi) * cosf(pi / (4.0f * alpha)));
        } else {
            v = (sinf(pi * t * (1.0f - alpha)) + 4.0f * alpha * t * cosf(pi * t * (1.0f + alpha))) / (pi * t * den);
        }
        h[k] = v;
        sum += v;
    }
    for (int k = 0; k <= kFirTaps / 2; k++)
        taps[k] = h[k] / sum;

    for (int f = 0; f < kNumFamilies; f++) {
        for (int k = 0; k < kSyncSyms; k++) {
            const unsigned dibit = (kVoiceSync[f][k >> 2] >> (6 - 2 * (k & 3))) & 3u;
            sgn[f][k]            = (dibit == 1u) ? 1.0f : -1.0f; /* 01 = +3, 11 = -3 */
        }
    }
    reset();
}

void Demod::reset()
{
    memset(hist, 0, sizeof(hist));
    hist_n  = kFirTaps - 1;
    phase   = 0;
    dc_slow = 0.0f;
    dc_init = false;
    memset(ring, 0, sizeof(ring));
    n         = 0;
    pk_active = false;
    pk_n      = 0;
    pk_fam    = 0;
    pk_rho2 = pk_c0 = pk_cm = pk_cp = pk_d = 0.0f;
    memset(c_prev, 0, sizeof(c_prev));
    holdoff_until = 0;
}

bool Demod::push(float x)
{
    /* reject NaN / Inf / absurd values so that they cannot poison the state */
    if (!(fabsf(x) < 1e30f))
        x = 0.0f;

    if (hist_n == kFirHist) {
        memmove(hist, hist + kFirHist - (kFirTaps - 1), (kFirTaps - 1) * sizeof(float));
        hist_n = kFirTaps - 1;
    }
    hist[hist_n++] = x;
    phase ^= 1;
    if (phase)
        return false;

    const float *p = hist + hist_n - kFirTaps;
    float acc      = taps[kFirTaps / 2] * p[kFirTaps / 2];
    for (int k = 0; k < kFirTaps / 2; k++)
        acc += taps[k] * (p[k] + p[kFirTaps - 1 - k]);

    /* Very slow DC removal (time constant 0.7 s). It only keeps the numbers
     * well conditioned; the real offset is estimated per burst. */
    if (!dc_init) {
        dc_slow = acc;
        dc_init = true;
    }
    dc_slow += (acc - dc_slow) * (1.0f / 16384.0f);
    ring[n & kRingMask] = acc - dc_slow;
    n++;
    return true;
}

bool Demod::search(SyncDet &det)
{
    const int64_t m = n - 1;
    float xs[kSyncSyms];
    float sx = 0.0f, sxx = 0.0f;
    const int64_t base = m - (int64_t)kSps * (kSyncSyms - 1);
    for (int k = 0; k < kSyncSyms; k++) {
        const float v = ring[(base + (int64_t)kSps * k) & kRingMask];
        xs[k]         = v;
        sx += v;
        sxx += v * v;
    }
    float c[kNumFamilies];
    for (int f = 0; f < kNumFamilies; f++) {
        float acc      = 0.0f;
        const float *s = sgn[f];
        for (int k = 0; k < kSyncSyms; k++)
            acc += s[k] * xs[k];
        c[f] = acc;
    }

    bool found = false;
    if (pk_active && m == pk_n + 1)
        pk_cp = c[pk_fam];

    const float var = sxx - sx * sx * (1.0f / kSyncSyms);
    if (var > 1e-4f * sxx && var > 1e-30f && m >= holdoff_until) {
        int bf   = 0;
        float bc = fabsf(c[0]);
        for (int f = 1; f < kNumFamilies; f++) {
            if (fabsf(c[f]) > bc) {
                bc = fabsf(c[f]);
                bf = f;
            }
        }
        const float rho2 = bc * bc / ((float)kSyncSyms * var);
        if (rho2 > kRho2Low && (!pk_active || rho2 > pk_rho2)) {
            pk_active = true;
            pk_n      = m;
            pk_fam    = (uint8_t)bf;
            pk_rho2   = rho2;
            pk_c0     = c[bf];
            pk_cm     = c_prev[bf];
            pk_cp     = 0.0f;
            pk_d      = sx * (1.0f / kSyncSyms);
        }
    }

    if (pk_active && m >= pk_n + 2) {
        const float sg = pk_c0 >= 0.0f ? 1.0f : -1.0f;
        const float a = sg * pk_cm, b = sg * pk_c0, cc = sg * pk_cp;
        const float den = a - 2.0f * b + cc;
        float delta     = 0.0f;
        if (den < 0.0f)
            delta = 0.5f * (a - cc) / den;
        if (delta > 0.5f)
            delta = 0.5f;
        if (delta < -0.5f)
            delta = -0.5f;
        det.t_end  = pk_n * kOne + (tq_t)(delta * 65536.0f);
        det.family = pk_fam;
        det.sign   = (int8_t)(pk_c0 >= 0.0f ? 1 : -1);
        det.rho2   = pk_rho2;
        det.g3     = b * (1.0f / kSyncSyms);
        det.d      = pk_d;
        pk_active  = false;
        holdoff_until = pk_n + 2 * kSps;
        found      = true;
    }
    for (int f = 0; f < kNumFamilies; f++)
        c_prev[f] = c[f];
    return found;
}

/* Interpolate the 144 symbols; t0 = time of the first CACH symbol. */
void Demod::sample(tq_t t0, float *x) const
{
    const int64_t i0 = t0 >> 16;
    const float mu   = (float)(t0 & 0xFFFF) * (1.0f / 65536.0f);
    const float cm   = -mu * (mu - 1.0f) * (mu - 2.0f) * (1.0f / 6.0f);
    const float c0   = (mu + 1.0f) * (mu - 1.0f) * (mu - 2.0f) * 0.5f;
    const float c1   = -(mu + 1.0f) * mu * (mu - 2.0f) * 0.5f;
    const float c2   = (mu + 1.0f) * mu * (mu - 1.0f) * (1.0f / 6.0f);
    for (int j = 0; j < kTotalSyms; j++) {
        const int64_t i = i0 + (int64_t)kSps * j;
        x[j] = cm * ring[(i - 1) & kRingMask] + c0 * ring[i & kRingMask] + c1 * ring[(i + 1) & kRingMask] +
               c2 * ring[(i + 2) & kRingMask];
    }
}

static inline float decide(float v)
{
    if (v > 0.0f)
        return v > (2.0f / 3.0f) ? 1.0f : (1.0f / 3.0f);
    return v < -(2.0f / 3.0f) ? -1.0f : -(1.0f / 3.0f);
}

/* Slicer error energy over the 132 burst symbols (levels normalised). */
static float slicer_cost(const float *x, float g, float d)
{
    const float ig = 1.0f / g;
    float j        = 0.0f;
    for (int i = kCachSyms; i < kTotalSyms; i++) {
        const float v = (x[i] - d) * ig;
        const float e = v - decide(v);
        j += e * e;
    }
    return j;
}

float Demod::sample_cost(tq_t t0, float g, float d, float *x) const
{
    sample(t0, x);
    return slicer_cost(x, g, d);
}

static void sync_levels(const float *x, const float *s, int sign, float &g, float &d)
{
    float sx = 0.0f, c = 0.0f;
    const float *p = x + kCachSyms + (kSyncEndSym - kSyncSyms + 1);
    for (int k = 0; k < kSyncSyms; k++) {
        sx += p[k];
        c += s[k] * p[k];
    }
    d = sx * (1.0f / kSyncSyms);
    g = (float)sign * c * (1.0f / kSyncSyms);
}

void Demod::slice(tq_t t_end, int family, int sign, float g, float d, Burst &out) const
{
    float x[kTotalSyms];
    const tq_t t0  = t_end - (tq_t)(kSyncEndSym + kCachSyms) * kSps * kOne;
    const tq_t step = kOne / 2;

    float j0 = 0.0f;
    if (family >= 0) {
        sample(t0, x);
        float gs, ds;
        sync_levels(x, sgn[family], sign, gs, ds);
        if (gs > 0.0f) {
            g = gs;
            d = ds;
        }
        j0 = slicer_cost(x, g, d);
    } else {
        j0 = sample_cost(t0, g, d, x);
    }
    if (!(g > 1e-20f))
        g = 1e-20f;

    /* timing refinement: parabola through the cost at -0.5 / 0 / +0.5 */
    const float jm = sample_cost(t0 - step, g, d, x);
    const float jp = sample_cost(t0 + step, g, d, x);
    float tau      = 0.0f;
    const float den = jm - 2.0f * j0 + jp;
    if (den > 1e-12f)
        tau = 0.25f * (jm - jp) / den;
    if (tau > 0.75f)
        tau = 0.75f;
    if (tau < -0.75f)
        tau = -0.75f;
    out.tau = tau;
    sample(t0 + (tq_t)(tau * 65536.0f), x);

    if (family >= 0) {
        float gs, ds;
        sync_levels(x, sgn[family], sign, gs, ds);
        if (gs > 0.0f) {
            g = gs;
            d = ds;
        }
    }

    /* decision directed least-squares refit of level and offset */
    {
        const float ig = 1.0f / g;
        float sa = 0.0f, saa = 0.0f, sx = 0.0f, sax = 0.0f;
        for (int i = kCachSyms; i < kTotalSyms; i++) {
            const float a = decide((x[i] - d) * ig);
            sa += a;
            saa += a * a;
            sx += x[i];
            sax += a * x[i];
        }
        const float nn  = (float)kBurstSyms;
        const float det = nn * saa - sa * sa;
        if (det > 1e-6f) {
            const float g2 = (nn * sax - sa * sx) / det;
            if (g2 > 0.6f * g && g2 < 1.6f * g) {
                d = (sx - g2 * sa) / nn;
                g = g2;
            }
        }
    }

    /* final decisions */
    const float ig = 1.0f / g;
    float j        = 0.0f;
    memset(out.bits, 0, sizeof(out.bits));
    memset(out.cach, 0, sizeof(out.cach));
    for (int i = 0; i < kTotalSyms; i++) {
        const float v = (x[i] - d) * ig;
        const float a = decide(v);
        unsigned dibit;
        if (a > 0.0f)
            dibit = a > 0.5f ? 1u : 0u;
        else
            dibit = a < -0.5f ? 3u : 2u;
        if (i < kCachSyms) {
            out.cach[i >> 2] |= (uint8_t)(dibit << (6 - 2 * (i & 3)));
        } else {
            const int k = i - kCachSyms;
            out.bits[k >> 2] |= (uint8_t)(dibit << (6 - 2 * (k & 3)));
            const float e = v - a;
            j += e * e;
        }
    }
    out.g   = g;
    out.d   = d;
    out.evm = sqrtf(j * (1.0f / kBurstSyms));
}

} // namespace dmrrx
