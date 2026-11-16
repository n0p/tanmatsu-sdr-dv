# dmr_rx

DMR (ETSI TS 102 361) receiver / decoder library. Input is the output of an FM
discriminator at 48 kS/s, output is events (sync, colour code, slot type, link
control, CSBK, talker alias, AMBE+2 frames) and, optionally, 8 kHz PCM.
Portable C++17 inside, plain C API, no dependencies besides libc / libm.
Receive only.

Licence: AGPL-3.0-only (own code), with vendored GPL-2.0-or-later and ISC code
in `third_party/` (see the `NOTICE.md` in each directory).

## API (`include/dmr_rx.h`)

```c
dmr_rx_config_t cfg;
dmr_rx_config_default(&cfg);
cfg.event_cb = on_event;            /* void on_event(void *user, const dmr_event_t *e) */
cfg.pcm_cb   = on_pcm;              /* optional: 160 samples, 8 kHz, per AMBE frame    */
dmr_rx_t *rx = dmr_rx_create(&cfg); /* the only allocation (cfg.alloc to override)     */

dmr_rx_process_fm(rx, samples, n);      /* float, any block size  */
dmr_rx_process_fm_s16(rx, samples, n);  /* int16, same thing      */

dmr_rx_status_t st;
dmr_rx_get_status(rx, &st);         /* pollable snapshot, also from another task */
dmr_rx_reset(rx);                   /* after retuning */
dmr_rx_destroy(rx);
```

Input contract: real samples at exactly 48 000 Hz, 12.5 kHz channel filter
before the discriminator, no de-emphasis and no audio high-pass / AGC after it.
Gain, DC offset and polarity are arbitrary: they are estimated per burst from
the sync word (polarity: `DMR_POLARITY_AUTO`, or force it in the config). The
int16 variant only scales by 1/32768.

Events (`dmr_event_t`, delivered synchronously from `dmr_rx_process_*`): every
event carries timeslot (1/2, `slot_known` says whether it comes from the
CACH TACT / the direct mode sync or is only a timing phase), sync type, colour
code, polarity, sample index of the burst and a quality record (sync bit
errors, FEC-corrected bits and bits covered, EVM).

| Event | Content |
|---|---|
| `SYNC_ACQUIRED` / `SYNC_LOST` | per slot; sync type BS / MS / direct mode TS1 / TS2, voice or data |
| `DATA_BURST` | data type from the slot type (Golay(20,8)), BPTC(196,96) payload |
| `VOICE_BURST` | burst index A..F, the three AMBE+2 frames raw (72 bit) and FEC-decoded (49 bit), per-frame error count and `bad` flag, `encrypted` |
| `LC` | full LC from voice LC header / terminator (RS(12,9)) or embedded LC (bursts B..E, 5 bit checksum): PF, FLCO, FID, service options, dst, src |
| `CSBK` | CRC-CCITT checked; opcode, FID, raw payload, name, ids where the common layout applies; MBC headers |
| `PI_HEADER`, `DATA_HEADER` | algorithm / key id / MI; data header addresses, SAP, blocks to follow |
| `DATA_PDU` | a data packet, put together from its header(s) and blocks: addresses, SAP, UDP ports, the user data, what it is (`kind`, `format`), and for text messages the text in UTF-8 |
| `SHORT_LC` | from the CACH (Hamming(17,12) rows + CRC-8), SLCO name |
| `TALKER_ALIAS` | FLCO 4..7 blocks assembled, 7 bit / 8 bit / UTF-8 / UTF-16 to UTF-8 |
| `CALL_START` / `CALL_UPDATE` / `CALL_END` | call state per slot incl. `late_entry`, `encrypted`, `emergency` |

Encrypted calls (privacy bit in the service options, or a PI header) are
flagged in `LC`, `VOICE_BURST`, the call events and the status; their frames
are never handed to the vocoder and the PCM callback is not called.

`dmr_vocoder_*()` gives stand-alone access to the speech decoder (49 bit frame
in, 160 samples out).

## Design

One pass, three layers:

1. **Front end** (`src/dmr_demod.cpp`): 81 tap root raised cosine matched
   filter evaluated at every second input sample (24 kS/s, 5 samples per
   symbol) into a ring buffer. For every new sample the 24 sync symbols ending
   there are correlated against the four sync families. Sync symbols are all
   outer symbols and the patterns are DC balanced, so correlation, mean and
   energy give a closed-form least-squares fit of level, offset and residual:
   the detector does not depend on gain, DC offset or polarity and needs no
   AGC, PLL or settling time. A burst is then sliced from the ring at the
   sync-derived instant (cubic interpolation, timing refined on the slicer
   error, levels refitted decision directed).
2. **Slot tracker** (`src/dmr_rx.cpp`): two TDMA slots, each with a predicted
   burst time (60 ms period, tracked to +-300 ppm). Bursts without sync
   (voice B..F) are sliced at the predicted time. Slot numbers come from
   majority-voted TACT bits.
   A slot is confirmed by FEC evidence before anything is reported, which is
   what keeps noise and non-DMR 4FSK from producing events.
3. **PDU layer**: FEC and PDU classes vendored from MMDVMHost, AMBE Golay
   decode from mbelib, interleave schedule from DSD.
4. **Packet data** (`src/dmr_data.cpp`, `src/dmr_pdu.cpp`): the blocks behind
   a data header (rate 1/2 BPTC, rate 3/4 trellis, rate 1) are collected per
   slot, in a fixed buffer of 64 blocks. The header says whether the blocks
   carry a serial number and a CRC-9 (confirmed data); radios do not always
   set it the way the standard reads, so both layouts are tried and the
   CRC-32 over the packet decides. A packet ends with its last announced
   block, or earlier when the transmission stops or turns to something else;
   it is then reported with the number of blocks missing. The content is
   read as Motorola TMS, Hytera TMP, the defined short data formats, a
   status, or one of the services known by their UDP port (ARS, LRRP,
   telemetry, ...); failing that, a run of UTF-16 or UTF-8 characters that
   makes up most of the packet is taken as text and marked as a guess
   (`format` "text? ...").

The rate 3/4 trellis coder and its Viterbi decoder are written for this
library from TS 102 361-1 annex B (state transition table, constellation,
interleaver); no code from MMDVMHost's `DMRTrellis.cpp` (GPL-2.0 only) or
dsd-fme is used. The same goes for the checksums and the message formats.

Why this architecture instead of porting DSD / dsd-fme / DSDcc as a whole:
their symbol recovery uses a running min/max level tracker and a free-running
symbol clock, which need settling time, and it is intertwined with file /
audio I/O, globals and `double` math. The parts worth reusing are the tables
and the block codes; those are reused. The burst-wise least-squares front end
is new code; on the recordings below it gives burst counts equal to or higher
than dsd-fme's on all but two, and fewer symbol errors.

### Upstream code

| Directory | Upstream | Licence | Used for |
|---|---|---|---|
| `third_party/mmdvmhost` | g4klx/MMDVMHost @ 6c849ab | GPL-2.0-or-later | BPTC(196,96), Golay(20,8), QR(16,7), Hamming, RS(12,9), CRC, slot type, EMB, full / embedded / short LC |
| `third_party/mbelib` | szechyjs/mbelib @ 9a04ed5 | ISC | AMBE 3600x2450 Golay / descrambling, speech decoder (ported to single precision) |
| `third_party/dsd` | szechyjs/dsd @ 59423fa | ISC | AMBE interleave tables (`dmr_const.h`) |

Changes are listed in each `NOTICE.md`. The host signal generator uses the
*encoder* halves of the MMDVMHost classes, so synthetic tests are not the
decoder run backwards.

## Building

ESP-IDF: the directory is a component (`CMakeLists.txt`), no `REQUIRES`.
`DMR_RX_WITH_VOICE` (CMake variable, default ON) builds the speech decoder.

Host:

```
make -C host            # library, tests, dmr_rx_file, dmr_gen, bench
make -C host test       # everything that needs no downloads
make -C host VOICE=0 BUILD=build_novoice test
host/build/dmr_rx_file [-q|-v] [-w out.s16] recording.wav|.s16
host/build/bench [seconds] [P4 slowdown factor]
```

Host builds use `-Wall -Wextra -Werror -Wdouble-promotion`; the library is
compiled with `-fno-exceptions -fno-rtti`.

## Test evidence

### Off-air capture from the Tanmatsu (primary acceptance test)

`testdata/dmr_430400000_fm_48k.s16` (8.0 s, hotspot on 430.400 MHz),
`dmr_rx_file` reading the file as it is:

| | dmr_rx | dsd-fme (same file) |
|---|---|---|
| polarity | normal (locked) | normal |
| colour code | 1 (260 bursts) | 1 |
| syncs | 155 (22 BS voice, 133 BS data), 0 losses, 0 sync bit errors | - |
| TS1 | 133 Idle bursts, all FEC-good | 113 Idle |
| TS2 | 128 voice bursts = 384 AMBE frames, 0 bad, 0 with Golay errors | 100 bursts = 300 frames, 141 with Golay errors |
| call | group call, late entry, TG 91, source 2780081, FLCO 0, FID 0, svc 0 | 2780081 > 91 |
| embedded LC | 10 voice LC + talker alias blocks (FLCO 4/5/6) | LC only |
| talker alias | "9H5JO Joe" | not decoded |
| short LC | SLCO 1 (activity update) | - |
| estimates | clock +49 ppm, deviation 0.077 of full scale, DC -0.004, FEC BER 0 | - |

DSDcc on the same file: `BS S1: .01 IDL S2: * VOX 02780081>G00000091`.
Both decoders start with the voice burst at 0.31 s and end with the last
one of the file; dsd-fme loses 28 voice and 20 idle bursts in between. Of
dsd-fme's 300 frames 152 are bit-identical to dmr_rx's; the others are
frames in which dsd-fme has symbol errors (see the next section).

### Public recordings against dsd-fme

22 recordings (8 native discriminator audio, 8 FM-demodulated here from
published IQ, 6 decoded from MP3), repeater, MS, direct mode, Hytera,
Motorola Connect Plus / Capacity Plus control channels, encrypted calls.
Reference: dsd-fme built from source, same WAV files.

- Colour code: identical on all 22.
- Source / destination id sets: identical on all 22.
- Timeslots with voice: identical on all.
- Good bursts per type (voice LC header, terminator, PI, CSBK, data header,
  rate 1/2, rate 3/4, idle): dmr_rx >= dsd-fme on 21 of 22; e.g. idle
  613 vs 388, CSBK 957 vs 862, terminators 46 vs 41.
- AMBE frames: dmr_rx >= dsd-fme on 21 of 22 (e.g. 1602 vs 1224, 5319 vs
  5316). The 49 bit frames were compared bit by bit: where both decoders
  receive a recording without bit errors they are identical (954 of 954,
  756 of 756, 486 of 486, 990 of 990). Where they are not, the 24 Golay
  protected information bits still agree and the difference is in the 25
  unprotected bits, on recordings where dsd-fme's symbol decisions are in
  error: frames with Golay errors, dsd-fme vs dmr_rx, are 141 of 300 vs 0
  of 384 on the Tanmatsu capture, 1715 of 5316 vs 0 of 5319 on
  `nivex_bs_tg2`, 79 vs 11 of 756 on `mp3_tdma_voice`; on no recording does
  dmr_rx have the higher share. A noise-free generated signal with known
  content shows the same (dmr_rx: all 1080 frames exact; dsd-fme: Golay
  errors in 503 of 936).
- The two rows where dmr_rx has fewer: `iq_sdrtrunk1803_dmo` 990 vs 1008 AMBE
  frames (weak signal, the 990 are identical), `mapper_c0013` voice LC header
  0 vs 3 (dsd-fme counts three headers that fail RS(12,9) and print as
  "Unknown LC FLCO=0x0E FID=0x29"; dmr_rx reports them with `payload_ok = 0`).

How to run: put 48 kHz mono 16 bit WAVs in a directory, run
`dsd-fme -fs [-xr] -i NAME.wav -o null -Z > REF/NAME.dsd-fme.log` for each and

```
python3 -I host/compare_dsdfme.py --bin host/build/dmr_rx_file --wav DIR --ref REF
```

The recordings are not redistributed here (no licence stated by most
publishers); the sources are listed at the end.

### Synthetic (`make -C host test`)

Generator: MMDVMHost encoders, 4FSK at 4800 symbols/s, RRC alpha 0.2,
+-648 / +-1944 Hz, 48 kS/s, noise added in the discriminator domain (SNR
figures are in 24 kHz bandwidth).

- `test_fec` (260 checks): every FEC / CRC block (AMBE Golay, slot type,
  EMB, CACH / TACT, BPTC(196,96) + RS(12,9) / CRC-CCITT PDUs, short LC,
  embedded LC, sync words) against vectors that do not come from this
  library (`host/test_vectors.h`: frames received off air with the values
  dsd-fme printed for them, PDUs dsd-fme printed for public recordings, the
  MMDVM idle and silence constants) and against the upstream encoders;
  correction up to the capability of each code.
- `test_rx` (289 checks): clean repeater signal: all 78 data and 66 voice
  bursts, all AMBE frames bit exact, burst position within 2 samples.
  Gain 0.01 .. 30, int16 scaling, DC +-0.3, inverted, +-50 and +-100 ppm
  clock offset, all combined: identical event sequences. Block sizes 1, 2,
  7, 160, 480, 1024, 4096, 48000 and random, on a noisy signal (4 dB):
  bit-identical events, status and PCM; int16 == float. MS, direct mode TS1 / TS2, colour codes 0..15.
  Late entry, two simultaneous calls, encrypted calls (flagged, no PCM),
  talker alias, short LC, CSBK, data headers, dropped input (1 .. 48000
  samples), reset, no heap use while processing. No events in 60 s each of
  silence, weak / strong / un-squelched noise, a 1 kHz tone, a 1200 baud
  square wave and random 4800 baud 4FSK without sync, and none with the
  wrong polarity forced.
- `test_data` (190 checks): rate 3/4 blocks and block / packet checksums
  against received data from the test suite of ok-dmrlib (AGPL-3.0; data
  only, none of its code): three trellis coded blocks, CRC-9 at all three rates,
  CRC-32, and the three bursts of a Motorola text message acknowledgement,
  which go through the modulator and the receiver and come out with a
  right CRC-32. Text payloads of Motorola and Hytera radios from the same
  source, and built ones for every format. Packets of all three rates,
  confirmed and unconfirmed, every payload length, a header that says the
  wrong layout, proprietary header, short data, status, a block lost in
  three ways, transmission cut short, a voice call cutting in, both
  timeslots, MS and direct mode. 60 000 random and damaged payloads through
  the interpreter: no overrun, always valid UTF-8, 0.2 % of random packets
  taken for text. Packets right of 12 at 9 dB and above: all, at every
  rate; 7 dB: all at rate 1/2 and 3/4, 2 to 3 at rate 1 (which has no
  error correction); 5 dB: 12, 9, 0. No packet with wrong content passed
  its CRC at any SNR, none came out of a minute of noise.
- SNR sweep (4 seeds, +-20 ppm): down to 11 dB everything exact; 9 dB: 791 of
  792 AMBE frames exact; 7 dB: 97 %; 5 dB: 82 %, 56 of 60 LCs; 3 dB: 52 %,
  45 of 60 LCs, all 8 calls still found. No wrong LC and no false call at any
  SNR.
- `test_voice`: the speech decoder against the original double precision
  mbelib (see below).

### CPU and memory

`host/build/bench 30` on the development host (x86-64, one core), blocks of
960 samples:

| Case | ns / sample | realtime | of a core 15x slower |
|---|---|---|---|
| noise, no signal | 32 - 34 | 620 - 650x | 2.3 - 2.4 % |
| repeater idle / one call / two calls, signalling only | 32 - 34 | 620 - 650x | 2.3 - 2.4 % |

The cost is dominated by the matched filter and the sync correlators, which
run on every sample; decoding bursts adds almost nothing. The vocoder comes on
top, per decoded call (figures in the vocoder section).

State: one allocation of `dmr_rx_state_size()` bytes: 17 904 without the
vocoder, 26 832 with it (5.5 KB of either for packet data). No allocation, no `double`, no 64 bit division
per sample in the processing path.

RISC-V: all sources compile with
`riscv32-esp-elf-gcc/g++ 15.2 -march=rv32imafc_zicsr_zifencei -mabi=ilp32f -O2 -Wall -Wextra -Werror -Wdouble-promotion`
with and without the vocoder; no double precision soft-float helper is
referenced. A scratch ESP-IDF 6.0 project for esp32p4 using this component
builds and links. Nothing was run on the device.

## Vocoder (`DMR_RX_WITH_VOICE`)

AMBE+2 (AMBE 3600x2450) decoder, a single precision port of mbelib's
synthesiser (details in `third_party/mbelib/NOTICE.md`): no `double`, no
`cosf` / `exp` / `pow` per sample (recurrence oscillators and tables, the only
libm call is `sqrtf`), no globals, 4460 bytes of state per timeslot.

Output: 160 samples of 8 kHz int16 per frame through `pcm_cb`, three calls
per voice burst, `pcm_gain` 1.0 = mbelib's scaling. Frames with `bad` set or
6 and more Golay errors repeat the previous parameters, the fourth such frame
in a row mutes (mbelib's rule).

Audio is only decoded for calls known to be clear: on late entry it is held
back until the embedded LC with the service options has arrived (at most 12
bursts, 0.72 s; 0.6 s in the Tanmatsu capture), and never started when the
privacy bit or a PI header was seen. The five recordings of the test set that
signal encryption produce no PCM.

Verification:

- `test_voice` (34 checks): 150 frames of a public recording against PCM of
  the original double precision mbelib embedded in the test (72 dB SNR, max
  difference 2 LSB), concealment / mute rules, silence frames, instance
  independence, gain, 2000 random frames without non-finite output.
- Outside `make test` (scratch harness, reference = upstream mbelib with only
  `rand()` replaced by the port's generator), all voice frames of 5
  recordings incl. the Tanmatsu capture (67.1 dB, max difference 7 LSB),
  8127 frames: 55.8 .. 67.1 dB SNR against the reference, frame decisions
  (ok / repeat / erasure / tone / mute) identical in every frame. The
  remaining difference is the reference's float rounding of the phase:
  against the reference with that one argument computed in double it is
  91 dB, max 1 LSB. Bit-identical PCM from gcc -O2 and clang -O3.
  3 million random frames: no non-finite output; ASan / UBSan clean.
- Not verified: execution on the target (compile only), and no listening
  test against a DVSI decoder (mbelib quality is the ceiling).

Cost: 13 us per frame on recorded speech (reference mbelib in double: 131 us),
32 us in `bench` on the generator's test frames; that is 1.0 .. 2.4 % of a
core 15 times slower, per decoded call:

| Case (`bench 30`, PCM callback set) | ns / sample | of a core 15x slower |
|---|---|---|
| voice on TS1, TS2 idle | 66 | 4.8 % |
| voice on both timeslots, both decoded | 100 | 7.2 % |

State with the vocoder: 26 832 bytes in total. Stack: the deepest chain
(`process_fm` .. parameter decode) is about 2.2 KB of frames on RV32, plus
what the callbacks use.

## Limits and known weak points

- Not run on the ESP32-P4. CPU figures are host measurements scaled by 15.
- 48 000 Hz only. Clock error is tracked to +-300 ppm (tested to +-100).
- Packet data: only the downlink that is received is put together; nothing
  is known of retransmissions (a confirmed packet that is sent again is
  reported again) and response packets are not evaluated. Unified data
  transport (UDT, used on Tier III control channels for short messages) is
  not decoded. Packets longer than 64 blocks are cut.
- The rate 3/4 code has 52 check bits for 144 data bits: a block with a few
  symbol errors can decode to another valid block (one of the three received
  test blocks does). Random bits come out within 10 bits of a code word in
  0.8 % of the cases, which is the limit used to tell a block from noise.
  What is right is decided by CRC-9 (confirmed) and CRC-32.
- Text formats: Motorola TMS, Hytera TMP and ETSI short data are implemented
  from public descriptions and tested with a few received payloads and with
  generated ones. Whether the CRC-32 of real Hytera packets with a
  proprietary header covers that header is not known (assumed: not).
  Encrypted payloads are not recognised as such. ASCII text in UTF-16BE at
  an odd offset reads the same as UTF-16LE one byte later; the guess takes
  LE and can put one stray character in front.
- CSBK / MBC: opcode, FID, raw payload, name and the common address layout.
  Tier III and vendor trunking messages are not interpreted further. No
  genuine Tier III control channel recording was available; the trunking
  recordings used are Connect Plus / Capacity Plus and MP3-derived.
- Vendor LCs (FID != 0, e.g. Hytera) are passed through as received;
  `dst_id` / `src_id` are only meaningful for the standard layouts.
- The slot number of MS-sourced bursts cannot be known (no CACH);
  `slot_known` is 0 and the two timing phases are numbered in order of
  appearance.
- After dropped input samples running calls are closed and picked up again as
  late entry; one burst may carry the wrong timeslot number before the TACT
  votes correct it. Three stations with conflicting timing on one frequency
  are sorted out within 6 bursts.
- No recording with voice on both timeslots at the same time was found; that
  case is covered by synthetic tests only.
- Below about 7 dB SNR (24 kHz bandwidth) voice frames start to differ from
  the transmitted ones; they are flagged by `errs` / `bad`.
- Vocoder: as in mbelib, tone frames are not synthesised (silence), and a
  frame flagged bad whose garbage happens to look like an erasure or tone
  frame (6 of 128 patterns) gives 20 ms of silence instead of a repeat.
  mbelib's output gain clips on very loud audio (0.08 % of the samples of one
  recording); lower `pcm_gain` if that matters.
- Encrypted audio is not decoded, by design. Basic privacy that is not
  signalled in the LC (some Hytera / Motorola scrambling) cannot be detected
  and comes out as garbled audio.

## Integration notes

- One task calls `dmr_rx_process_*`, `dmr_rx_reset` and
  `dmr_rx_set_pcm_slot_mask`. Callbacks run in that task, inside the call.
  `dmr_rx_get_status` may be called from any task (snapshot, published once
  per process call).
- Do not call back into the same receiver from a callback (except
  `dmr_rx_get_status`).
- Feed the discriminator output unfiltered. Scale does not matter (tested:
  rad/sample times 0.01 .. 30, and int16 with 32767 = pi rad/sample); avoid
  clipping. Either polarity works.
- Call `dmr_rx_reset` after retuning.
- The component sets `-O2` for itself (the CPU figures were measured at
  `-O2`).

## Recording sources

Native discriminator audio: f4exb/dsdcc `samples/dmr_it_8.dis`;
szechyjs/dsd-samples `dmr/dmr_1_48000.raw`, `dmr_2_48000.raw`;
Shahar373/dmr-system-mapper `C0013 .../discriminator.wav`; nivex
`gqrx-20140918-211010.flac` (szechyjs/dsd issue 43); lwvmobile/dsd-fme issue
94 `MSMS_DMR_Simplex.wav`; f4exb/dsdcc issue 43 (two Hytera recordings).
IQ, FM-demodulated for this test: sigidwiki `DMR.zip`; mrbaranovskiy/dmr-light
`dmr_test.wav` and two `tests/fixtures/*.cs16`; DSheirer/sdrtrunk issues 1746
(two files) and 1803. MP3: sigidwiki `ConplusCC.mp3`,
`DMR_Control_Channel_454580kHz_AF.mp3`, `DMRMOTOTRBO_Sound.mp3`, `DMR.mp3`,
`DMR-RC4.mp3`, `DMR_TDMA_signal.mp3`.
