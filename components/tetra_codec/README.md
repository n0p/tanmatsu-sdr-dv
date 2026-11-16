# tetra_codec — TETRA speech decoder (opt-in)

Turns the 137-bit speech frames that `tetra_rx` delivers into audio: 240 samples (30 ms) at 8 kHz
per frame. The codec is ACELP at 4.567 kbit/s as specified in ETSI EN 300 395-2.

## The codec is not in this repository

The decoder is the **ETSI reference C code**, which ETSI publishes together with the standard as
`en_30039502v010301p0.zip` (EN 300 395-2 V1.3.1, on etsi.org). ETSI's copyright does not allow
redistributing it, so:

- nothing of it is checked in here, and the archive name is in `.gitignore`;
- **a firmware binary built with it must not be passed on** (app repository, releases, ...). It is
  for your own device.

To build with voice, download the archive in a browser and either put it in the project root or
point `TETRA_CODEC_ZIP` (environment or CMake variable) at it, then build as usual. The build
prints which way it went:

```
-- tetra_codec: TETRA voice built in from .../en_30039502v010301p0.zip (binary is not redistributable)
-- tetra_codec: no ETSI codec archive, TETRA voice is not built in
```

Without the archive the component is a stub (`tetra_codec_available()` returns false) and the app
shows calls without playing them.

## How it is built

`prepare.py` unpacks the `C-CODE/` directory of the archive into the build directory (lower-case
file names, Unix line ends; nothing else is changed). `src/tetra_codec.c` then includes the
decoder files into one translation unit, compiled with `-fwhole-program`, and arranges the few
things the 1990s code needs on a current compiler and a small target — see the comment at its
top (fixed-width types, `round()`, no `exit()` on a bad division, all globals local).

Cost on the ESP32-P4: 8.6 KB code, 9.7 KB of internal RAM. The time per frame was not measured on
its own; the TETRA receiver takes about 26 % of one core without a call and 29 to 32 % while one
is decoded.

## API

```c
bool tetra_codec_available(void);
void tetra_codec_reset(void);                       // start of a talk spurt
void tetra_codec_decode(const uint8_t bits[137], bool bad_frame, int16_t pcm[240]);
```

One decoder instance (the reference code keeps its state in globals); call it from one task.
`bad_frame` is for frames that were stolen for signalling or failed the CRC: the decoder then
conceals the gap from its previous state.

## Tests (on the PC)

```sh
make -C host test          # needs the archive, see ZIP= in host/Makefile
host/build/tetra_voice_file ../../testdata/tetra_391262500_voicecall_72k.cs16 call.wav
```

- `test_codec`: speech-like test signals are encoded with the reference coder and decoded twice,
  by the reference decoder built the way ETSI's makefile builds it (file by file) and by this
  wrapper. The outputs must be bit-identical for clean frames, bad-frame runs and random bits.
- `tetra_voice_file`: decodes the speech of a 72 kHz I/Q recording (as made with
  `tools/tsdr.py cap ch`) to a WAV file through `tetra_rx` and this wrapper, i.e. the firmware's
  path. It also runs the reference *channel* decoder on the soft bits of every traffic slot and
  compares the result with the frames `tetra_rx` delivered, which checks the codec bit order of
  `tetra_rx` against the reference. On the recording in `testdata/` (20 s of a clear group call):
  540 of 540 frames identical.
