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

/* Modified for dmr_rx (tanmatsu-sdr), see NOTICE.md: this file holds the
 * speech parameter decoder of upstream ambe3600x2450.c (the FEC functions
 * are in ambe3600x2450_ecc.c, the dump / frame / short variants are gone):
 *  - single precision only, no libm call other than sqrtf (): the DCT
 *    cosines, 2 pi f0 and log2 (L) come from tables, exp () is a local
 *    single precision implementation;
 *  - mbe_processAmbe2450Dataf: the caller decides whether the frame is to be
 *    replaced by a repeat (upstream: errs2 > 3), no error string, the
 *    random generator / work state is passed in, the frame type is returned;
 *  - mbe_decodeAmbe2450Parms: the read of prev_mp->log2Ml[prev_mp->L + 1]
 *    with L == 56 (one element past the array upstream) is made valid;
 *  - floating point contraction (fused multiply-add) is switched off. */

#ifdef DMR_RX_WITH_VOICE

#include <math.h>

#include "mbelib.h"
#include "ambe3600x2450_const.h"

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

/**
 * \return exp (x), single precision, relative error about 1e-7. x is
 * limited to +-40 (upstream: unlimited, double), far outside of the range
 * of decodable speech, to keep the sums of squared amplitudes finite.
 */
static float
mbe_expf (float x)
{
  union
  {
    float f;
    uint32_t u;
  } s;
  float kf, r, p;
  int k;

  if (!(x > -40.0f))
    {
      x = -40.0f;
    }
  else if (x > 40.0f)
    {
      x = 40.0f;
    }
  kf = x * 1.44269504f;         /* x / ln (2) */
  k = (int) (kf + ((kf < 0.0f) ? -0.5f : 0.5f));
  kf = (float) k;
  r = (x - (kf * 0.693359375f)) - (kf * -2.12194440e-4f);       /* |r| <= ln (2) / 2 */
  p = 1.0f + (r * (1.0f + (r * (0.5f + (r * (1.66666667e-1f + (r * (4.16666667e-2f + (r * (8.33333333e-3f + (r * (1.38888889e-3f + (r * 1.98412698e-4f)))))))))))));
  s.u = (uint32_t) (k + 127) << 23;     /* 2^k, |k| <= 58 */
  return (p * s.f);
}

int
mbe_decodeAmbe2450Parms (const char *ambe_d, mbe_parms * cur_mp, mbe_parms * prev_mp)
{

  int ji, i, j, k, l, L, m, kmax;
  unsigned char intkl[57];
  int b0, b1, b2, b3, b4, b5, b6, b7, b8;
  float f0, Cik[5][7], flokl, deltal[57];
  float Sum42, Sum43, Tl[57], Gm[9], Ri[9], sum, c1, c2;
  int silence;
  int Ji[5], jl;
  float deltaGamma, BigGamma;
  float unvc, rconst;
  const float *hoc[5];
  const float *idct;
  char d[49];

  silence = 0;
  L = 0;
  f0 = 0.0f;

  for (i = 0; i < 49; i++)
    {
      d[i] = ambe_d[i] & 1;
    }

  // copy repeat from prev_mp
  cur_mp->repeat = prev_mp->repeat;

  // decode fundamental frequency w0 from b0
  b0 = 0;
  b0 |= d[0]<<6;
  b0 |= d[1]<<5;
  b0 |= d[2]<<4;
  b0 |= d[3]<<3;
  b0 |= d[37]<<2;
  b0 |= d[38]<<1;
  b0 |= d[39];
  if ((b0 >= 120) && (b0 <= 123)) // if w0 bits are 1111000, 1111001, 1111010 or 1111011, frame is erasure
    {
      return (2);
    }
  else if ((b0 == 124) || (b0 == 125)) // if w0 bits are 1111100 or 1111101, frame is silence
    {
      silence = 1;
      cur_mp->w0 = 0.196349546f;        // 2 pi / 32
      f0 = 1.0f / 32.0f;
      L = 14;
      cur_mp->L = 14;
      for (l = 1; l <= L; l++)
        {
          cur_mp->Vl[l] = 0;
        }
    }
  else if ((b0 == 126) || (b0 == 127)) // if w0 bits are 1111110 or 1111111, frame is tone
    {
      return (3);
    }

  if (silence == 0)
    {
      // w0 from specification document
      f0 = AmbeW0table[b0];
      cur_mp->w0 = AmbeW0rad[b0];       // f0 * 2 pi
    }

  unvc = 0.2046f / sqrtf (cur_mp->w0);

  // decode L
  if (silence == 0)
    {
      // L from specification document
      // lookup L in tabl3
      L = AmbeLtable[b0];
      cur_mp->L = L;
    }

  // decode V/UV parameters
  // load b1 from ambe_d
  b1 = 0;
  b1 |= d[4]<<4;
  b1 |= d[5]<<3;
  b1 |= d[6]<<2;
  b1 |= d[7]<<1;
  b1 |= d[35];

  if (silence == 0)
    {
      for (l = 1; l <= L; l++)
        {
          // jl from specification document
          jl = (int) ((float) l * 16.0f * f0);
          cur_mp->Vl[l] = AmbeVuv[b1][jl & 7];
        }
    }

  // decode gain vector
  // load b2 from ambe_d
  b2 = 0;
  b2 |= d[8]<<4;
  b2 |= d[9]<<3;
  b2 |= d[10]<<2;
  b2 |= d[11]<<1;
  b2 |= d[36];

  deltaGamma = AmbeDg[b2];
  cur_mp->gamma = deltaGamma + (0.5f * prev_mp->gamma);


  // decode PRBA vectors
  Gm[1] = 0.0f;

  // load b3 from ambe_d
  b3 = 0;
  b3 |= d[12]<<8;
  b3 |= d[13]<<7;
  b3 |= d[14]<<6;
  b3 |= d[15]<<5;
  b3 |= d[16]<<4;
  b3 |= d[17]<<3;
  b3 |= d[18]<<2;
  b3 |= d[19]<<1;
  b3 |= d[40];
  Gm[2] = AmbePRBA24[b3][0];
  Gm[3] = AmbePRBA24[b3][1];
  Gm[4] = AmbePRBA24[b3][2];

  // load b4 from ambe_d
  b4 = 0;
  b4 |= d[20]<<6;
  b4 |= d[21]<<5;
  b4 |= d[22]<<4;
  b4 |= d[23]<<3;
  b4 |= d[41]<<2;
  b4 |= d[42]<<1;
  b4 |= d[43];
  Gm[5] = AmbePRBA58[b4][0];
  Gm[6] = AmbePRBA58[b4][1];
  Gm[7] = AmbePRBA58[b4][2];
  Gm[8] = AmbePRBA58[b4][3];

  // compute Ri
  for (i = 1; i <= 8; i++)
    {
      sum = 0.0f;
      for (m = 1; m <= 8; m++)
        {
          // am * Gm[m] * cos ((pi * (m - 1) * (i - 0.5)) / 8)
          sum = sum + (Gm[m] * AmbeRiCos[i - 1][m - 1]);
        }
      Ri[i] = sum;
    }

  // generate first to elements of each Ci,k block from PRBA vector
  rconst = 0.353553385f;        // 1 / (2 * sqrt (2))
  Cik[1][1] = 0.5f * (Ri[1] + Ri[2]);
  Cik[1][2] = rconst * (Ri[1] - Ri[2]);
  Cik[2][1] = 0.5f * (Ri[3] + Ri[4]);
  Cik[2][2] = rconst * (Ri[3] - Ri[4]);
  Cik[3][1] = 0.5f * (Ri[5] + Ri[6]);
  Cik[3][2] = rconst * (Ri[5] - Ri[6]);
  Cik[4][1] = 0.5f * (Ri[7] + Ri[8]);
  Cik[4][2] = rconst * (Ri[7] - Ri[8]);

  // decode HOC

  // load b5 from ambe_d
  b5 = 0;
  b5 |= d[24]<<4;
  b5 |= d[25]<<3;
  b5 |= d[26]<<2;
  b5 |= d[27]<<1;
  b5 |= d[44];

  // load b6 from ambe_d
  b6 = 0;
  b6 |= d[28]<<3;
  b6 |= d[29]<<2;
  b6 |= d[30]<<1;
  b6 |= d[45];

  // load b7 from ambe_d
  b7 = 0;
  b7 |= d[31]<<3;
  b7 |= d[32]<<2;
  b7 |= d[33]<<1;
  b7 |= d[46];

  // load b8 from ambe_d
  b8 = 0;
  b8 |= d[34]<<2;
  b8 |= d[47]<<1;
  b8 |= d[48];

  // lookup Ji
  Ji[1] = AmbeLmprbl[L][0];
  Ji[2] = AmbeLmprbl[L][1];
  Ji[3] = AmbeLmprbl[L][2];
  Ji[4] = AmbeLmprbl[L][3];

  // Load Ci,k with the values from the HOC tables
  // there appear to be a couple typos in eq. 37 so we will just do what makes sense
  // (3 <= k <= Ji and k<=6)
  hoc[1] = AmbeHOCb5[b5];
  hoc[2] = AmbeHOCb6[b6];
  hoc[3] = AmbeHOCb7[b7];
  hoc[4] = AmbeHOCb8[b8];
  for (i = 1; i <= 4; i++)
    {
      for (k = 3; (k <= Ji[i]) && (k <= 6); k++)
        {
          Cik[i][k] = hoc[i][k - 3];
        }
    }

  // inverse DCT each Ci,k to give ci,j (Tl)
  // (Ci,k is zero for k > 6: those terms are not computed)
  l = 1;
  for (i = 1; i <= 4; i++)
    {
      ji = Ji[i];
      kmax = (ji < 6) ? ji : 6;
      idct = &AmbeIdctCos[AMBE_IDCT_OFFSET (ji)];
      for (j = 1; j <= ji; j++)
        {
          sum = Cik[i][1];
          for (k = 2; k <= kmax; k++)
            {
              // ak * Cik[i][k] * cos ((pi * (k - 1) * (j - 0.5)) / ji)
              sum = sum + (Cik[i][k] * idct[k - 2]);
            }
          idct += 5;
          Tl[l] = sum;
          l++;
        }
    }

  // determine log2Ml by applying ci,j to previous log2Ml

  // fix for when L > L(-1)
  if (cur_mp->L > prev_mp->L)
    {
      for (l = (prev_mp->L) + 1; l <= cur_mp->L; l++)
        {
          prev_mp->Ml[l] = prev_mp->Ml[prev_mp->L];
          prev_mp->log2Ml[l] = prev_mp->log2Ml[prev_mp->L];
        }
    }
  prev_mp->log2Ml[0] = prev_mp->log2Ml[1];
  prev_mp->Ml[0] = prev_mp->Ml[1];

  // Part 1
  // (intkl[l] + 1 is prev_mp->L + 1 for l == L, upstream reads past the end
  // of log2Ml for prev_mp->L == 56, which is PHIl[0] == 0 there; here
  // log2Ml has an element [57] that stays 0)
  Sum43 = 0.0f;
  for (l = 1; l <= cur_mp->L; l++)
    {

      // eq. 40
      flokl = ((float) prev_mp->L / (float) cur_mp->L) * (float) l;
      intkl[l] = (unsigned char) ((int) (flokl));
      // eq. 41
      deltal[l] = flokl - (float) intkl[l];
      // eq 43
      Sum43 = Sum43 + (((1.0f - deltal[l]) * prev_mp->log2Ml[intkl[l]]) + (deltal[l] * prev_mp->log2Ml[intkl[l] + 1]));
    }
  Sum43 = ((0.65f / (float) cur_mp->L) * Sum43);

  // Part 2
  Sum42 = 0.0f;
  for (l = 1; l <= cur_mp->L; l++)
    {
      Sum42 += Tl[l];
    }
  Sum42 = Sum42 / (float) cur_mp->L;
  BigGamma = cur_mp->gamma - AmbeHalfLog2L[cur_mp->L] - Sum42;  // gamma - 0.5 * log2 (L) - Sum42

  // Part 3
  for (l = 1; l <= cur_mp->L; l++)
    {
      c1 = (0.65f * (1.0f - deltal[l]) * prev_mp->log2Ml[intkl[l]]);
      c2 = (0.65f * deltal[l] * prev_mp->log2Ml[intkl[l] + 1]);
      cur_mp->log2Ml[l] = Tl[l] + c1 + c2 - Sum43 + BigGamma;
      // inverse log to generate spectral amplitudes
      if (cur_mp->Vl[l] == 1)
        {
          cur_mp->Ml[l] = mbe_expf (0.693f * cur_mp->log2Ml[l]);
        }
      else
        {
          cur_mp->Ml[l] = unvc * mbe_expf (0.693f * cur_mp->log2Ml[l]);
        }
    }

  return (0);
}

/*
 * repeat != 0: conceal the frame by repeating the previous parameters
 * (upstream: errs2 > 3). As upstream, the frame is decoded first and an
 * erasure / tone frame takes precedence over the repeat.
 */
int
mbe_processAmbe2450Dataf (float *aout_buf, int repeat, const char ambe_d[49], mbe_parms * cur_mp, mbe_parms * prev_mp, mbe_parms * prev_mp_enhanced, mbe_synth * syn)
{

  int bad, ret;

  bad = mbe_decodeAmbe2450Parms (ambe_d, cur_mp, prev_mp);
  if (bad == 2)
    {
      // Erasure frame
      ret = MBE_FRAME_ERASURE;
      cur_mp->repeat = 0;
    }
  else if (bad == 3)
    {
      // Tone Frame
      ret = MBE_FRAME_TONE;
      cur_mp->repeat = 0;
    }
  else if (repeat)
    {
      ret = MBE_FRAME_REPEAT;
      mbe_useLastMbeParms (cur_mp, prev_mp);
      cur_mp->repeat++;
    }
  else
    {
      ret = MBE_FRAME_OK;
      cur_mp->repeat = 0;
    }

  if (bad == 0)
    {
      if (cur_mp->repeat <= 3)
        {
          mbe_moveMbeParms (cur_mp, prev_mp);
          mbe_spectralAmpEnhance (cur_mp);
          mbe_synthesizeSpeechf (aout_buf, cur_mp, prev_mp_enhanced, syn);
          mbe_moveMbeParms (cur_mp, prev_mp_enhanced);
        }
      else
        {
          ret = MBE_FRAME_MUTE;
          mbe_synthesizeSilencef (aout_buf);
          mbe_initMbeParms (cur_mp, prev_mp, prev_mp_enhanced);
        }
    }
  else
    {
      mbe_synthesizeSilencef (aout_buf);
      mbe_initMbeParms (cur_mp, prev_mp, prev_mp_enhanced);
    }
  return (ret);
}

#else /* DMR_RX_WITH_VOICE */

typedef int dmrrx_mbe_no_ambe2450_t;    /* ISO C forbids an empty translation unit */

#endif
