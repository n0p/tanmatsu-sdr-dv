# mbelib (AMBE 3600x2450 FEC and speech decoder)

- Upstream: https://github.com/szechyjs/mbelib
- Commit: 9a04ed5c78176a9965f3d43f7aa1b1f5330e771f
- Licence: ISC (see `COPYRIGHT`, unmodified; every file keeps its header)
- Copyright (C) 2010 mbelib Author

Only the AMBE 3600x2450 (DMR, "AMBE+2") decode path is vendored. IMBE, AMBE
3600x2400, the 48 kHz output variants, the debug dump code and the
`mbe_processAmbe3600x2450Frame*` wrappers are not included. Every modified
file says "Modified for dmr_rx" below its licence header.

All symbols are renamed with a `dmrrx_` prefix (by `#define`s in `mbelib.h`)
so that another component that links mbelib cannot collide.

## Files

| File | Upstream file | Change |
|---|---|---|
| `COPYRIGHT` | `COPYRIGHT` | Unmodified. |
| `ecc.c` | `ecc.c` | Only the Golay(23,12) decoder (`mbe_checkGolayBlock`, `mbe_golay2312`) is kept. Function-local statics removed (re-entrant). |
| `ecc_const.h` | `ecc_const.h` | Hamming tables (IMBE only) removed, the Golay tables are `static const`. |
| `ambe3600x2450_ecc.c` | `ambe3600x2450.c` | The integer-only functions `mbe_eccAmbe3600x2450C0`, `mbe_eccAmbe3600x2450Data`, `mbe_demodulateAmbe3600x2450Data`, code unchanged, moved to their own file so that the frame FEC builds without the speech synthesiser. |
| `ambe3600x2450.c` | `ambe3600x2450.c` | Speech parameter decoder (`mbe_decodeAmbe2450Parms`, `mbe_processAmbe2450Dataf`). Single precision only: the DCT cosines, 2 pi f0 and log2(L) come from tables, `exp()` is a local single precision routine, no libm call except `sqrtf`. `mbe_processAmbe2450Dataf` takes a `repeat` flag from the caller instead of `errs` / `errs2`, has no error string, gets its work / random state passed in and returns the frame decision (`MBE_FRAME_OK / REPEAT / ERASURE / TONE / MUTE`); the handling of each case is upstream's. The read of `prev_mp->log2Ml[prev_mp->L + 1]` with L = 56 (one past the array upstream) is made valid. Whole file inside `#ifdef DMR_RX_WITH_VOICE`. |
| `ambe3600x2450_const.h` | `ambe3600x2450_const.h` | Tables are `static const` and single precision (`f` suffix), the small integer tables are `unsigned char`, the unused alternate `AmbeVuv` table is dropped. Added at the end, computed from the upstream formulas: `AmbeW0rad`, `AmbeHalfLog2L`, `AmbeRiCos`, `AmbeIdctCos`. |
| `mbelib.c` | `mbelib.c` | Reduced to what the AMBE decoder needs; `uvquality` fixed to 3 (the DSD default). Single precision only. `rand()` is replaced by a 32 bit LCG kept in the new per-instance `mbe_synth` state (numbers drawn in upstream's order), so there are no globals and the output is reproducible. `mbe_synthesizeSpeechf`: the `cosf()` per harmonic and sample is replaced by a second order recurrence oscillator started from exactly reduced phases; zero / unity sections of the synthesis window are not multiplied; harmonics with zero amplitude are skipped. `mbe_spectralAmpEnhance`: same weights, written so that intermediates cannot overflow, `powf(x, 0.25)` is two square roots. `mbe_floattoshort` takes a linear gain (1.0 = upstream's x7), clamps to +-32760 and maps NaN to 0. Whole file inside `#ifdef DMR_RX_WITH_VOICE`. |
| `mbelib_const.h` | `mbelib_const.h` | Only the 8 kHz synthesis window is kept, `static const`, single precision literals. |
| `mbelib.h` | `mbelib.h` | Prototypes of the functions above, `mbe_parms` in single precision, new `mbe_synth` state struct, frame decision constants, symbol prefix `#define`s. |

Floating point contraction (fused multiply-add) is switched off by pragma in
`mbelib.c` and `ambe3600x2450.c`, so the decoded samples do not depend on the
compiler's `-ffp-contract` default or on the target.

The port is tested against the unmodified double precision upstream code in
`host/test_voice.cpp` (reference output embedded there).
