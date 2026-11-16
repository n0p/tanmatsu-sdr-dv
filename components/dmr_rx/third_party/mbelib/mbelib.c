/*
 * Copyright (C) 2010 mbelib Author
 * GPG Key ID: 0xEA5EFE2C (9E7A 5527 9CDC EBF7 BF1B  D772 4F98 E863 EA5E FE2C)
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND ISC DISCLAIMS ALL WARRANTIES WITH
 * REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS.  IN NO EVENT SHALL ISC BE LIABLE FOR ANY SPECIAL, DIRECT,
 * INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
 * LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE
 * OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/* Modified for dmr_rx (tanmatsu-sdr), see NOTICE.md:
 *  - reduced to what the AMBE 3600x2450 decoder needs (no version string,
 *    no short / 48 kHz variants), uvquality is fixed to 3 (the DSD default);
 *  - single precision only: no double arithmetic and no libm call other
 *    than sqrtf ();
 *  - re-entrant: rand () is replaced by a 32 bit LCG kept in mbe_synth, the
 *    random numbers are drawn in the same order as upstream draws them;
 *  - mbe_synthesizeSpeechf: every cosf () per harmonic and sample is
 *    replaced by a second order recurrence (y[n] = 2 cos (w) y[n-1] - y[n-2])
 *    started from exactly reduced phases, the zero / unity sections of the
 *    synthesis window are not multiplied out, harmonics with zero amplitude
 *    are skipped; the output is the same signal up to rounding;
 *  - mbe_spectralAmpEnhance: same weights, written so that the intermediate
 *    values cannot overflow, powf (x, 0.25) is two square roots;
 *  - mbe_floattoshort takes an additional linear gain and maps NaN to 0;
 *  - floating point contraction (fused multiply-add) is switched off. */

#ifdef DMR_RX_WITH_VOICE

#include <math.h>
#include <string.h>

#include "mbelib.h"
#include "mbelib_const.h"

/* No contraction of a * b + c into fused multiply-add: the results are then
 * the same on every IEEE 754 target (the host tests and the ESP32-P4 give
 * the same samples), independent of the compiler's -ffp-contract default.
 * (clang disregards the pragma when -ffp-contract=fast is given explicitly;
 * the output then differs in rounding only.) */
#if defined(__clang__)
#pragma clang fp contract(off)
#elif defined(__GNUC__)
#pragma GCC optimize ("fp-contract=off")
#endif

#define MBE_N           160             /* samples per frame */
#define MBE_RAMP_START  56              /* first sample of the window ramps */
#define MBE_RAMP_END    MBE_SYNTH_SPAN  /* first sample after the window ramps */

#define MBE_PI          3.14159274f     /* (float) M_PI */
#define MBE_RAD2TURNS31 341782637.788f  /* 2^31 / (2 pi) */
#define MBE_TURNS2RAD   1.4629180792671596e-9f  /* 2 pi / 2^32 */

/* constants of upstream mbe_synthesizeSpeechf for uvquality == 3 */
#define MBE_UVSINE      3.69452786f     /* 1.3591409 * e */
#define MBE_UVRAND      2.0f
#define MBE_UVTHRESHOLD 2.12057495f     /* 2700 * pi / 4000 */
#define MBE_UVSTEP      0.333333343f    /* 1 / uvquality */
#define MBE_UVOFFSET    0.333333343f    /* uvstep * (uvquality - 1) / 2 */
#define MBE_QFACTOR     0.366204083f    /* log (uvquality) / uvquality */
#define MBE_UVQUALITY   3

#define MBE_LCG_A       1664525u
#define MBE_LCG_C       1013904223u
#define MBE_LCG_A3      (MBE_LCG_A * MBE_LCG_A * MBE_LCG_A)
#define MBE_LCG_C3      ((MBE_LCG_A * MBE_LCG_A + MBE_LCG_A + 1u) * MBE_LCG_C)
#define MBE_RAND_SEED   0x2545f491u

/**
 * \return A pseudo-random float between [0.0, 1.0).
 */
static inline float
mbe_rand (uint32_t * rs)
{
  *rs = (*rs * MBE_LCG_A) + MBE_LCG_C;
  return ((float) (*rs >> 8) * (1.0f / 16777216.0f));
}

/**
 * \return A pseudo-random float between [-pi, +pi].
 */
static inline float
mbe_rand_phase (uint32_t * rs)
{
  return mbe_rand (rs) * (MBE_PI * 2.0f) - MBE_PI;
}

/**
 * \return The sum of three consecutive mbe_rand () values.
 */
static inline float
mbe_rand3 (uint32_t * rs)
{
  uint32_t s, a;

  s = (*rs * MBE_LCG_A) + MBE_LCG_C;
  a = s >> 8;
  s = (s * MBE_LCG_A) + MBE_LCG_C;
  a += s >> 8;
  s = (s * MBE_LCG_A) + MBE_LCG_C;
  a += s >> 8;
  *rs = s;
  return ((float) a * (1.0f / 16777216.0f));
}

/**
 * \return The generator state after n more calls of mbe_rand ().
 */
static uint32_t
mbe_rand_skip (uint32_t s, unsigned int n)
{
  uint32_t a, c, acc_a, acc_c;

  a = MBE_LCG_A;
  c = MBE_LCG_C;
  acc_a = 1u;
  acc_c = 0u;
  while (n != 0u)
    {
      if (n & 1u)
        {
          acc_a = acc_a * a;
          acc_c = (acc_c * a) + c;
        }
      c = (a + 1u) * c;
      a = a * a;
      n >>= 1;
    }
  return ((acc_a * s) + acc_c);
}

/**
 * Exact conversion of an angle in radians to a phase in units of
 * 2^-32 turns (modulo one turn). Works for any finite float: the phases of
 * upstream (PSIl / PHIl) grow without bound during a call and cosf () of
 * the original reduces them exactly, so the port has to do the same.
 * 64 bits of 1 / (2 pi) are used: exact to 2^-32 turns for |x| < 2^24,
 * beyond that the float has no fractional turn information left anyway.
 */
static uint32_t
mbe_turns_exact (float x)
{
  union
  {
    float f;
    uint32_t u;
  } v;
  uint32_t m, r;
  uint64_t lo, hi;
  int e, sh;

  v.f = x;
  e = (int) ((v.u >> 23) & 0xffu);
  if (e == 0xff)
    {
      return (0u);              /* inf / nan */
    }
  m = (v.u & 0x7fffffu) | 0x800000u;
  /* |x| = m * 2^(e - 150); 1 / (2 pi) = 0x28be60db9391054a * 2^-64
     phase = (m * 0x28be60db9391054a) >> (182 - e) (mod 2^32) */
  lo = (uint64_t) m * 0x9391054au;
  hi = ((uint64_t) m * 0x28be60dbu) + (lo >> 32);       /* product = hi * 2^32 + (lo & 0xffffffff) */
  sh = 182 - e;
  if (sh >= 96)
    {
      r = 0u;                   /* |x| < 2^-37 (and denormals) */
    }
  else if (sh >= 32)
    {
      r = (uint32_t) (hi >> (sh - 32));
    }
  else if (sh > 0)
    {
      r = (uint32_t) (hi << (32 - sh)) | ((uint32_t) lo >> sh);
    }
  else if (sh > -32)
    {
      r = (uint32_t) lo << (-sh);
    }
  else
    {
      r = 0u;
    }
  return ((v.u >> 31) ? (0u - r) : r);
}

/**
 * Angle in radians to phase in units of 2^-32 turns, fast path for the
 * small angles (frequencies, random phases).
 */
static inline uint32_t
mbe_turns (float x)
{
  if ((x > -6.0f) && (x < 6.0f))
    {
      return ((uint32_t) ((int32_t) (x * MBE_RAD2TURNS31)) << 1);
    }
  return (mbe_turns_exact (x));
}

/**
 * \return cos of a phase given in units of 2^-32 turns, error < 1e-7.
 */
static inline float
mbe_cos_turns (uint32_t p)
{
  uint32_t q;
  float r, r2, y;

  q = (p + 0x20000000u) >> 30;  /* nearest multiple of a quarter turn */
  r = (float) ((int32_t) (p - (q << 30))) * MBE_TURNS2RAD;      /* remainder, -pi/4 .. pi/4 */
  r2 = r * r;
  if (q & 1u)
    {
      /* +- sin (r) */
      y = r * (1.0f + (r2 * (-1.66666667e-1f + (r2 * (8.33333333e-3f + (r2 * (-1.98412698e-4f + (r2 * 2.75573192e-6f))))))));
      return ((q & 2u) ? y : -y);
    }
  /* +- cos (r) */
  y = 1.0f + (r2 * (-0.5f + (r2 * (4.16666667e-2f + (r2 * (-1.38888889e-3f + (r2 * 2.48015873e-5f)))))));
  return ((q & 2u) ? -y : y);
}

void
mbe_moveMbeParms (const mbe_parms * cur_mp, mbe_parms * prev_mp)
{
  /* upstream copies every field that is left in this port */
  *prev_mp = *cur_mp;
}

void
mbe_useLastMbeParms (mbe_parms * cur_mp, const mbe_parms * prev_mp)
{
  *cur_mp = *prev_mp;
}

void
mbe_initMbeParms (mbe_parms * cur_mp, mbe_parms * prev_mp, mbe_parms * prev_mp_enhanced)
{

  int l;

  memset (prev_mp, 0, sizeof (*prev_mp));
  prev_mp->w0 = 0.09378f;
  prev_mp->L = 30;
  prev_mp->gamma = 0.0f;
  for (l = 0; l <= 56; l++)
    {
      prev_mp->Ml[l] = 0.0f;
      prev_mp->Vl[l] = 0;
      prev_mp->log2Ml[l] = 0.0f;        // log2 of 1 == 0
      prev_mp->PHIl[l] = 0.0f;
      prev_mp->PSIl[l] = 1.57079637f;   // pi / 2
    }
  prev_mp->log2Ml[57] = 0.0f;
  prev_mp->repeat = 0;
  mbe_moveMbeParms (prev_mp, cur_mp);
  mbe_moveMbeParms (prev_mp, prev_mp_enhanced);
}

void
mbe_initSynth (mbe_synth * syn)
{
  memset (syn, 0, sizeof (*syn));
  syn->rand_state = MBE_RAND_SEED;
}

void
mbe_spectralAmpEnhance (mbe_parms * cur_mp)
{

  float Rm0, Rm1, rho, den, scale, t, Wl, cosw0l[57];
  int l;
  float sum, gamma, M;

  Rm0 = 0.0f;
  Rm1 = 0.0f;
  for (l = 1; l <= cur_mp->L; l++)
    {
      cosw0l[l] = mbe_cos_turns (mbe_turns (cur_mp->w0 * (float) l));
      M = cur_mp->Ml[l] * cur_mp->Ml[l];
      Rm0 = Rm0 + M;
      Rm1 = Rm1 + (M * cosw0l[l]);
    }

  if (!((Rm0 > 0.0f) && (Rm0 < 1.0e38f)))
    {
      // nothing to enhance (upstream: no amplitude is modified, gamma is 1), or out of range
      return;
    }

  /* upstream: Wl = sqrt (Ml) * (0.96 pi (Rm0^2 + Rm1^2 - 2 Rm0 Rm1 cos (w0 l)) / (w0 Rm0 (Rm0^2 - Rm1^2)))^0.25
     here numerator and denominator are divided by Rm0^2 */
  rho = Rm1 / Rm0;
  den = cur_mp->w0 * Rm0 * (1.0f - (rho * rho));
  if (den > 0.0f)
    {
      scale = (0.96f * MBE_PI) / den;
      for (l = 1; l <= cur_mp->L; l++)
        {
          if ((cur_mp->Ml[l] != 0.0f) && ((8 * l) > cur_mp->L))
            {
              t = (1.0f + (rho * rho)) - (2.0f * rho * cosw0l[l]);
              if (t < 0.0f)
                {
                  t = 0.0f;
                }
              Wl = sqrtf (cur_mp->Ml[l] * sqrtf (scale * t));

              if (Wl > 1.2f)
                {
                  cur_mp->Ml[l] = 1.2f * cur_mp->Ml[l];
                }
              else if (Wl < 0.5f)
                {
                  cur_mp->Ml[l] = 0.5f * cur_mp->Ml[l];
                }
              else
                {
                  cur_mp->Ml[l] = Wl * cur_mp->Ml[l];
                }
            }
        }
    }

  // generate scaling factor
  sum = 0.0f;
  for (l = 1; l <= cur_mp->L; l++)
    {
      M = cur_mp->Ml[l];
      sum += (M * M);
    }
  if (sum == 0.0f)
    {
      gamma = 1.0f;
    }
  else
    {
      gamma = sqrtf (Rm0 / sum);
    }

  // apply scaling factor
  for (l = 1; l <= cur_mp->L; l++)
    {
      cur_mp->Ml[l] = gamma * cur_mp->Ml[l];
    }
}

void
mbe_synthesizeSilencef (float *aout_buf)
{

  int n;

  for (n = 0; n < MBE_N; n++)
    {
      aout_buf[n] = 0.0f;
    }
}

/**
 * Voiced harmonic of the previous frame (eq 131 / 133-1):
 * aout[n] += Ws[n + N] * M * cos (w * n + phi), n = 0 .. 104
 * (Ws[n + N] is 1 up to n = 55 and 0 from n = 105).
 */
static void
mbe_voicedPrev (float *aout_buf, float M, uint32_t phi, uint32_t w)
{
  float k, y, y1, y2;
  int n;

  k = 2.0f * mbe_cos_turns (w);
  y1 = M * mbe_cos_turns (phi);         /* n = 0 */
  y2 = M * mbe_cos_turns (phi - w);     /* n = -1 */
  aout_buf[0] += y1;
  for (n = 1; n < MBE_RAMP_START; n++)
    {
      y = (k * y1) - y2;
      y2 = y1;
      y1 = y;
      aout_buf[n] += y;
    }
  for (; n < MBE_RAMP_END; n++)
    {
      y = (k * y1) - y2;
      y2 = y1;
      y1 = y;
      aout_buf[n] += Ws[n + MBE_N] * y;
    }
}

/**
 * Voiced harmonic of the current frame (eq 132 / 133-2):
 * aout[n] += Ws[n] * M * cos (w * (n - N) + phi), n = 56 .. 159
 * (Ws[n] is 0 up to n = 55 and 1 from n = 105). Runs backwards from the
 * end of the frame, where phi is defined.
 */
static void
mbe_voicedCur (float *aout_buf, float M, uint32_t phi, uint32_t w)
{
  float k, y, y1, y2;
  int n;

  k = 2.0f * mbe_cos_turns (w);
  y1 = M * mbe_cos_turns (phi - w);     /* n = N - 1 */
  y2 = M * mbe_cos_turns (phi);         /* n = N */
  aout_buf[MBE_N - 1] += y1;
  for (n = MBE_N - 2; n >= MBE_RAMP_END; n--)
    {
      y = (k * y1) - y2;
      y2 = y1;
      y1 = y;
      aout_buf[n] += y;
    }
  for (; n >= MBE_RAMP_START; n--)
    {
      y = (k * y1) - y2;
      y2 = y1;
      y1 = y;
      aout_buf[n] += Ws[n] * y;
    }
}

/**
 * Unvoiced multisine mix of one harmonic over len samples starting at
 * sample n0 of the frame:
 * buf[j] = g * sum (i = 0 .. 2) cos (w0 * (n0 + j) * (l + i * uvstep - uvoffset) + rphase[i])  [+ gn * buf[j]]
 * On entry buf holds the sum of the three random numbers per sample when
 * noisy is set.
 */
static void
mbe_unvoiced (float *buf, int len, int n0, float g, float gn, int noisy, float w0, int l, const uint32_t rphase[MBE_UVQUALITY])
{
  float k[MBE_UVQUALITY], y1[MBE_UVQUALITY], y2[MBE_UVQUALITY], y, s;
  uint32_t w, p;
  int i, j;

  for (i = 0; i < MBE_UVQUALITY; i++)
    {
      w = mbe_turns (w0 * ((float) l + ((float) i * MBE_UVSTEP) - MBE_UVOFFSET));
      p = rphase[i] + ((uint32_t) n0 * w);
      k[i] = 2.0f * mbe_cos_turns (w);
      y1[i] = g * mbe_cos_turns (p - w);
      y2[i] = g * mbe_cos_turns (p - (2u * w));
    }
  if (noisy)
    {
      for (j = 0; j < len; j++)
        {
          y = (k[0] * y1[0]) - y2[0];
          y2[0] = y1[0];
          y1[0] = y;
          s = y;
          y = (k[1] * y1[1]) - y2[1];
          y2[1] = y1[1];
          y1[1] = y;
          s += y;
          y = (k[2] * y1[2]) - y2[2];
          y2[2] = y1[2];
          y1[2] = y;
          s += y;
          buf[j] = s + (gn * buf[j]);
        }
    }
  else
    {
      for (j = 0; j < len; j++)
        {
          y = (k[0] * y1[0]) - y2[0];
          y2[0] = y1[0];
          y1[0] = y;
          s = y;
          y = (k[1] * y1[1]) - y2[1];
          y2[1] = y1[1];
          y1[1] = y;
          s += y;
          y = (k[2] * y1[2]) - y2[2];
          y2[2] = y1[2];
          y1[2] = y;
          s += y;
          buf[j] = s;
        }
    }
}

/**
 * Draws the random numbers of the unvoiced part of one harmonic in the
 * order upstream calls mbe_rand () (per sample: uvquality values for the
 * previous frame's parameters if pn, then uvquality values for the current
 * frame's if cn). The per sample sums are stored in np (samples 0 .. 104)
 * and nc (samples 56 .. 159) where those are not NULL; values that would be
 * multiplied by a zero window or a zero amplitude are skipped.
 */
static uint32_t
mbe_unvoicedNoise (uint32_t rs, float *np, float *nc, int pn, int cn)
{
  int n;

  if (!pn)
    {
      np = NULL;
    }
  if (!cn)
    {
      nc = NULL;
    }

  // samples 0 .. 55: only the previous frame's window is not zero
  if (np != NULL)
    {
      for (n = 0; n < MBE_RAMP_START; n++)
        {
          np[n] = mbe_rand3 (&rs);
          if (cn)
            {
              rs = (rs * MBE_LCG_A3) + MBE_LCG_C3;
            }
        }
    }
  else
    {
      rs = mbe_rand_skip (rs, (unsigned int) (MBE_RAMP_START * MBE_UVQUALITY * ((pn ? 1 : 0) + (cn ? 1 : 0))));
    }

  // samples 56 .. 104: both
  for (n = MBE_RAMP_START; n < MBE_RAMP_END; n++)
    {
      if (np != NULL)
        {
          np[n] = mbe_rand3 (&rs);
        }
      else if (pn)
        {
          rs = (rs * MBE_LCG_A3) + MBE_LCG_C3;
        }
      if (nc != NULL)
        {
          nc[n - MBE_RAMP_START] = mbe_rand3 (&rs);
        }
      else if (cn)
        {
          rs = (rs * MBE_LCG_A3) + MBE_LCG_C3;
        }
    }

  // samples 105 .. 159: only the current frame's window is not zero
  if (nc != NULL)
    {
      for (n = MBE_RAMP_END; n < MBE_N; n++)
        {
          if (pn)
            {
              rs = (rs * MBE_LCG_A3) + MBE_LCG_C3;
            }
          nc[n - MBE_RAMP_START] = mbe_rand3 (&rs);
        }
    }
  else
    {
      rs = mbe_rand_skip (rs, (unsigned int) ((MBE_N - MBE_RAMP_END) * MBE_UVQUALITY * ((pn ? 1 : 0) + (cn ? 1 : 0))));
    }

  return (rs);
}

void
mbe_synthesizeSpeechf (float *aout_buf, mbe_parms * cur_mp, mbe_parms * prev_mp, mbe_synth * syn)
{

  int i, l, n, maxl;
  int numUv;
  int cV, pV, cn, pn;
  float cw0, pw0, cw0l, pw0l;
  float cM, pM, g;
  float *pbuf, *cbuf;
  uint32_t rphase[MBE_UVQUALITY], rphase2[MBE_UVQUALITY];
  uint32_t rs;

  const int N = MBE_N;

  rs = syn->rand_state;
  pbuf = syn->work[0];
  cbuf = syn->work[1];

  // count number of unvoiced bands
  numUv = 0;
  for (l = 1; l <= cur_mp->L; l++)
    {
      if (cur_mp->Vl[l] == 0)
        {
          numUv++;
        }
    }

  cw0 = cur_mp->w0;
  pw0 = prev_mp->w0;

  // init aout_buf
  for (n = 0; n < N; n++)
    {
      aout_buf[n] = 0.0f;
    }

  // eq 128 and 129
  if (cur_mp->L > prev_mp->L)
    {
      maxl = cur_mp->L;
      for (l = prev_mp->L + 1; l <= maxl; l++)
        {
          prev_mp->Ml[l] = 0.0f;
          prev_mp->Vl[l] = 1;
        }
    }
  else
    {
      maxl = prev_mp->L;
      for (l = cur_mp->L + 1; l <= maxl; l++)
        {
          cur_mp->Ml[l] = 0.0f;
          cur_mp->Vl[l] = 1;
        }
    }

  // update phil from eq 139,140
  for (l = 1; l <= 56; l++)
    {
      cur_mp->PSIl[l] = prev_mp->PSIl[l] + ((pw0 + cw0) * ((float) (l * N) / 2.0f));
      if (l <= (int) (cur_mp->L / 4))
        {
          cur_mp->PHIl[l] = cur_mp->PSIl[l];
        }
      else
        {
          cur_mp->PHIl[l] = cur_mp->PSIl[l] + (((float) numUv * mbe_rand_phase (&rs)) / (float) cur_mp->L);
        }
    }

  for (l = 1; l <= maxl; l++)
    {
      cw0l = (cw0 * (float) l);
      pw0l = (pw0 * (float) l);
      cV = (cur_mp->Vl[l] != 0);
      pV = (prev_mp->Vl[l] != 0);
      cM = cur_mp->Ml[l];
      pM = prev_mp->Ml[l];

      if (cV && pV)
        {
          // eq 133-1
          if (pM != 0.0f)
            {
              mbe_voicedPrev (aout_buf, pM, mbe_turns_exact (prev_mp->PHIl[l]), mbe_turns (pw0l));
            }
          // eq 133-2
          if (cM != 0.0f)
            {
              mbe_voicedCur (aout_buf, cM, mbe_turns_exact (cur_mp->PHIl[l]), mbe_turns (cw0l));
            }
          continue;
        }

      // init random phase: upstream draws the phases of the unvoiced
      // previous frame part first, then those of the current frame part
      if (!pV)
        {
          for (i = 0; i < MBE_UVQUALITY; i++)
            {
              rphase[i] = mbe_turns (mbe_rand_phase (&rs));
            }
        }
      if (!cV)
        {
          for (i = 0; i < MBE_UVQUALITY; i++)
            {
              rphase2[i] = mbe_turns (mbe_rand_phase (&rs));
            }
        }

      // random numbers of the bands above uvthreshold
      pn = (!pV) && (pw0l > MBE_UVTHRESHOLD);
      cn = (!cV) && (cw0l > MBE_UVTHRESHOLD);
      if (pn || cn)
        {
          rs = mbe_unvoicedNoise (rs, (pM != 0.0f) ? pbuf : NULL, (cM != 0.0f) ? cbuf : NULL, pn, cn);
        }

      if (cV && (cM != 0.0f))
        {
          // eq 132
          mbe_voicedCur (aout_buf, cM, mbe_turns_exact (cur_mp->PHIl[l]), mbe_turns (cw0l));
        }

      if (pM != 0.0f)
        {
          if (pV)
            {
              // eq 131
              mbe_voicedPrev (aout_buf, pM, mbe_turns_exact (prev_mp->PHIl[l]), mbe_turns (pw0l));
            }
          else
            {
              // unvoiced multisine mix
              g = MBE_UVSINE * pM * MBE_QFACTOR;
              mbe_unvoiced (pbuf, MBE_RAMP_END, 0, g, g * ((pw0l - MBE_UVTHRESHOLD) * MBE_UVRAND), pn, pw0, l, rphase);
              for (n = 0; n < MBE_RAMP_START; n++)
                {
                  aout_buf[n] += pbuf[n];
                }
              for (; n < MBE_RAMP_END; n++)
                {
                  aout_buf[n] += Ws[n + N] * pbuf[n];
                }
            }
        }

      if ((!cV) && (cM != 0.0f))
        {
          // unvoiced multisine mix
          g = MBE_UVSINE * cM * MBE_QFACTOR;
          mbe_unvoiced (cbuf, N - MBE_RAMP_START, MBE_RAMP_START, g, g * ((cw0l - MBE_UVTHRESHOLD) * MBE_UVRAND), cn, cw0, l, rphase2);
          for (n = MBE_RAMP_START; n < MBE_RAMP_END; n++)
            {
              aout_buf[n] += Ws[n] * cbuf[n - MBE_RAMP_START];
            }
          for (; n < N; n++)
            {
              aout_buf[n] += cbuf[n - MBE_RAMP_START];
            }
        }
    }

  syn->rand_state = rs;
}

void
mbe_floattoshort (const float *float_buf, int16_t *aout_buf, float gain)
{

  int i;
  float audio, again;

  again = 7.0f * gain;
  for (i = 0; i < MBE_N; i++)
    {
      audio = again * float_buf[i];
      if (audio > 32760.0f)
        {
          audio = 32760.0f;
        }
      else if (audio < -32760.0f)
        {
          audio = -32760.0f;
        }
      else if (audio != audio)
        {
          audio = 0.0f;         /* NaN */
        }
      aout_buf[i] = (int16_t) (audio);
    }
}

#else /* DMR_RX_WITH_VOICE */

typedef int dmrrx_mbe_no_voice_t;       /* ISO C forbids an empty translation unit */

#endif
