# tetra_rx

TETRA (ETSI EN 300 392-2) TMO downlink receiver and decoder in portable C11:
complex baseband in, cell identity / broadcast / signalling / speech codec
frames out. No dependencies besides libm. Licence: AGPL-3.0-or-later (the
channel coding and PDU tables come from osmo-tetra, see
`third_party/osmo-tetra/NOTICE`).

## API (`include/tetra_rx.h`)

```c
tetra_rx_config_t cfg = { .event_cb = on_event, .user = ctx, .flags = 0 };
tetra_rx_t *rx = tetra_rx_create(&cfg);          /* the only allocation */
tetra_rx_process_cf32(rx, iq, n_complex);        /* or tetra_rx_process_cs16 */
tetra_rx_get_status(rx, &st);                    /* same task, or from the callback */
n = tetra_rx_get_constellation(rx, pts, 256);    /* last symbols as the detector sees them */
tetra_rx_reset(rx);                              /* after retuning */
tetra_rx_destroy(rx);
```

Input: interleaved I,Q at 72 000 Hz (4 samples per symbol), one 25 kHz
downlink carrier within about +-3 kHz of 0 Hz, any block size, any scale
(`level_dbfs` assumes full scale 1.0 / 32768). The library does the channel
filtering (RRC, alpha 0.35), level normalisation, carrier and timing recovery.
A loose pre-filter is enough, with one caveat: whatever decimates to 72 kHz
must suppress carriers at +-75 kHz, they alias to -+3 kHz.

Events (synchronous callback from `tetra_rx_process_*`; pointers in an event
are valid during the call only):

| event | content |
|---|---|
| `SYNC` | burst synchronisation gained / lost, carrier offset |
| `BSCH` | SYNC PDU: MCC, MNC, colour code, TN/FN/MN, ... |
| `SYSINFO` | BNCH: main carrier, DL/UL frequency, LA, service bits, ... |
| `AACH` | ACCESS-ASSIGN per slot (opt-in flag) |
| `SIGNAL` | one MAC PDU: MAC-RESOURCE (address, encryption mode, channel allocation, slot grant), MAC-FRAG/END reassembled, MAC-BROADCAST; LLC type and FCS result; MLE protocol discriminator; CMCE (D-SETUP, D-CONNECT, D-TX GRANTED/CEASED, D-RELEASE, D-SDS-DATA incl. text, D-STATUS, ...) decoded; MM and the other protocols to PDU type; the raw SDU bits are always attached |
| `TRAFFIC` | TCH/S slot: 2 x 137 codec bits, bad frame indicators, stolen flags, usage marker, encryption state, type-4 soft bits |
| `BLOCK`, `BURST` | raw type-1 blocks / demodulated slots (opt-in, diagnostics) |

`tetra_rx_status_t`: lock, carrier offset (Hz), sample clock offset (ppm),
level, SNR, training sequence BER, per-channel CRC ok/fail counters, cell
identity, TDMA time, SYSINFO, per-timeslot usage and encryption state.
`locked` means burst synchronisation; `time.valid` and the counters tell that
blocks are actually being decoded.

A PDU the parser does not know or that is inconsistent (length, missing or
surplus bits) is reported with its type and raw bits and nothing else.
Encrypted PDUs are reported with their MAC header only; encrypted traffic is
only flagged (`enc_state`). There is no decryption code.

One receiver must be used from one task at a time (no locking, no globals
apart from a constant table built in `tetra_rx_create`). The library never
prints, asserts or aborts; after `tetra_rx_create` it never allocates.

## Design

| part | file | origin |
|---|---|---|
| modem: NCO, 41-tap RRC, O&M timing + cubic interpolator, differential detector, training sequence burst sync, carrier loop, TDMA flywheel | `src/tetra_modem.c` | written for this component |
| lower MAC: descrambling, de-interleaving, depuncturing, CRC, AACH, BSCH | `src/tetra_lmac.c` + osmo-tetra `lower_mac/`, `phy/tetra_burst.c` | glue new; tables / encoders / burst layout from osmo-tetra |
| soft-decision Viterbi (16 states, RCPC and TCH/S) and RM(30,14) decoder | `src/tetra_viterbi.c`, `src/tetra_lmac.c` | new (osmo-tetra uses libosmocore, not vendored) |
| upper MAC: PDU walk, fragment reassembly, LLC, FCS, MLE | `src/tetra_umac.c` + osmo-tetra `tetra_*_pdu.c` | new, PDU header parsers and name tables from osmo-tetra |
| CMCE element parser, SDS text | `src/tetra_cmce.c` | new |
| TCH/S channel decoder | `src/tetra_tch.c` + osmo-tetra `tch_reordering.c` | new, reordering table from osmo-tetra (two table errors fixed) |

All state is in one `struct tetra_rx`; the hot path is float only.

### Speech codec hook

The ACELP codec (EN 300 395-2 reference source) is not included and must not
be added to this repository: its ETSI licence does not allow redistribution.
Two hand-over points exist in `tetra_rx_ev_traffic_t`:

- `frame[i][137]`: channel-decoded codec bits of the two speech frames with
  `bfi[i]`; this is the input of a speech decoder (ETSI `sdecoder`).
- `soft` / `n_soft`: the 432 descrambled type-4 soft bits, the input of the
  ETSI channel decoder (`cdecoder`). osmo-tetra writes them as 690 int16 per
  slot (`block[115*i] = 0x6b21 + i` for i = 0..5, then the bits as +-127 at
  offsets 1, 116, 231 (114 each) and 346 (90)); the sign convention of `soft`
  is the same. `tetra_rx_file --dump-soft` writes them to a file.

The sibling component `../tetra_codec` consumes the first hand-over point: it builds the ETSI
decoder from the archive when the user supplies it. Its host tool `tetra_voice_file` also feeds
the second one to the ETSI channel decoder and compares the result with `frame[i]`; on the real
call in `testdata/` all 540 frames agree bit for bit, so the bit order of `frame[i]` is the
codec's.

## Tests

```
make -C host test     # unit tests + synthetic end-to-end tests, no files needed
make -C host bench    # CPU cost
make -C host riscv    # compile check with the ESP32-P4 toolchain, -Werror
host/build/tetra_rx_file --format cs16 --rate 72000 FILE     # decode a recording
host/build/tetra_rx_file --format cu8 --rate 1152000 --shift -288000 FILE
```

`tetra_rx_file` reads cf32 / cs16 / cu8 / cs8 / WAV at any rate (it resamples
to 72 kHz) and prints every event and the final status. To compare with
osmo-tetra: `tetra_rx_file --dump-bits out.bits FILE`, then run osmo-tetra's
`tetra-rx out.bits` and count its `CRC COMP: 0x1d0f OK` / `WRONG` lines.

**Unit tests** (`host/test_unit.c`, 1024 checks): CRC-16, scrambler against the
spec recurrence, RM(30,14) (minimum distance 8, all patterns of up to 3 errors
tried are corrected, 4 errors are rejected), interleavers, RCPC + Viterbi on
the osmo-tetra `conv_enc_test` vectors (every single error or erasure in a 432
bit block corrected), a complete osmo-tetra-built burst, BSCH, TCH/S coding,
TDMA time, CMCE and MAC parsers including truncated and oversized PDUs,
fragment reassembly, LLC FCS.

**Synthetic end-to-end** (`host/test_e2e.c`): a TETRA cell generator
(`host/tetra_gen.c`: osmo-tetra's encoder functions, its own PDU writer and
TCH/S encoder) through a channel model into the library.

- clean signal: every generated PDU, AACH, SYSINFO and speech frame recovered
  bit-exactly (e.g. 557/557 messages, 1393/1393 AACH, 220/220 TCH/S slots);
  identical events for block sizes 1, 7, 1024, 4096, random and for the cs16 API
- AWGN: message error rate 0.20 / 0.033 / 0.002 / 0 at Eb/N0 4 / 5 / 6 / >= 7 dB;
  raw BER follows differential DQPSK theory; no sync loss down to 4 dB
- carrier offset -2.5 .. +3 kHz: no errors, estimate within 3.5 Hz
- sample clock +-100 ppm: no errors, estimate within about 3 ppm
- acquisition from 40 random starts: burst sync <= 0.07 s, cell identity
  0.35 s average, 1.0 s worst
- 8-bit input down to 1 LSB rms, clipping up to +6 dB over full scale
- adjacent carrier at +-25 kHz: no errors up to +40 dB (float input), 8-bit
  input limited by quantisation at about +40 dB
- signal gaps: up to 3 slots without loss of lock, longer gaps re-lock with the
  right TDMA time; noise, CW, unstructured DQPSK and silence give no events
- encrypted call: PDUs reported header-only, traffic flagged encrypted

The generator shares the osmo-tetra tables with the decoder, so these tests
prove self-consistency and robustness, not conformance. Conformance evidence
is the real recordings below.

## Cost

| | |
|---|---|
| heap | one block, `sizeof(struct tetra_rx)` = 35248 bytes on RV32 |
| static | .data 0, .bss 76 bytes (constant tables built at start) |
| code + constants | about 35 kB on RV32 at -O2, before linker garbage collection |
| stack | `tetra_rx_process_cf32` 1216 bytes, `_cs16` 1232 bytes worst case (GCC `-fstack-usage` + call graph, RV32), plus libm leaf calls; the event callback is entered at most 1120 bytes deep |
| host CPU (x86-64) | 40 ns per sample locked (346 x real time), 48 ns searching; channel decoding and parsing 7.8 us per burst (about 1/5 of the locked cost) |
| ESP32-P4 | 25-26 % of one core, locked or searching (measured in the application) |

## Limits

- TMO downlink, pi/4-DQPSK, 25 kHz only. No uplink, DMO, TEDS, no packet data
  (SNDCP) content, no SCH/P8 etc.
- Speech: channel decoding only, no codec. The order of the 137 bits handed to
  the codec follows osmo-tetra's (corrected) table and EN 300 395-2 as read; it
  could not be checked against a codec. If the first half slot is stolen the
  speech frame in the second half is delivered as soft bits only (`bfi` set).
- No encrypted recording was available: the encryption flags are tested with
  synthetic signals only.
- MM, SNDCP and supplementary services are reported to PDU type level; CMCE
  optional elements beyond those listed in the header are left in the raw bits.
- The AACH code has no CRC: at very low SNR a wrong ACCESS-ASSIGN is accepted
  now and then (about 2 % of blocks with 6 bit errors), which can produce a
  spurious traffic or signalling attempt; those fail their own CRC.
- Tested with static channels only (AWGN, offsets, interferers); no fading
  model, no equaliser.
