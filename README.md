# Tanmatsu SDR — RTL-SDR receiver with DMR and TETRA decoding

An SDR receiver app for the [Tanmatsu](https://docs.tanmatsu.cloud) (ESP32-P4). An RTL-SDR dongle
goes into the USB-A host port; the app shows a spectrum and waterfall, demodulates NFM / AM / SSB /
WFM and decodes two digital modes on the device itself:

| Mode | What it does |
|---|---|
| NFM, AM, WFM | analogue audio |
| LSB, USB | single sideband, audio 0.3 – 2.7 kHz with AGC, tuning in 10 Hz steps |
| DMR | repeater, hotspot, simplex and direct mode signals: colour code, both timeslots, talk group and source IDs, talker alias, CSBK, text and status messages (Motorola, Hytera and ETSI short data formats), encryption flag, and voice (AMBE+2 decoded to the speaker) |
| TETRA | TMO downlink: cell identity (MCC / MNC / colour code / location area), main carrier, air-encryption flag, what each of the four timeslots is used for, and the clear signalling: call set-up and release, SDS text and status messages, mobility management. Voice of clear calls if the build includes the ETSI speech codec, which is opt-in (see [TETRA voice](#tetra-voice)). |

This is a new app, not a continuation of the "Tanmatsu SDR" app in the app store
(`fi.oh1w.tanmatsu-sdr`): that one has no public source. It uses the same USB driver,
[`esp-rtl-sdr`](https://github.com/hardcoreerik/esp-rtl-sdr), and installs next to it under its own
name (`nl.tanmatsu.sdr-dv`, "SDR DMR/TETRA").

## Keys

| Key | Action |
|---|---|
| LEFT / RIGHT | tune by one step (hold to repeat) |
| UP / DOWN | change the step (10 Hz … 1 MHz) |
| `0`-`9` `.` then RETURN | enter a frequency in MHz (ESC cancels) |
| TAB / Fn+TAB | next / previous mode (Shift+TAB works too) |
| `N` `A` `L` `U` `W` `D` `T` | NFM / AM / LSB / USB / WFM / DMR / TETRA |
| `+` / `-` | RF gain up / down (from automatic: one step from the last manual gain) |
| `G` | automatic gain on / off |
| F3 (yellow square) or `M`, then `0`-`9` | load a memory of the current bank |
| F3 `S` then `0`-`9` | save frequency and mode to a memory of the current bank |
| F3 then TAB / Fn+TAB | next / previous memory bank (0 – 9), then a number or `S` |
| `Z` | wide / narrow spectrum (450 kHz / 56 kHz across the screen; 112 kHz in WFM) |
| SPACE | NFM, AM, LSB, USB, WFM: squelch on / off. DMR, TETRA: which timeslot is heard, AUTO → 1 → 2 (→ 3 → 4) → AUTO |
| `S` | squelch level: `+` / `-` now move the level, `S` again gives them back to the gain |
| F2 (orange triangle) | [settings](#settings) |
| F4 (green circle) | all keys on one screen |
| F5 (blue trilobe) | the [messages](#messages) received last; UP / DOWN scroll, F5 or ESC closes |
| F6 (purple rhombus) | [record](#recording) the audio to the SD card; F6 again stops |
| Fn+F6 | [screenshot](#screenshots) to the SD card |
| volume keys | audio volume in steps of 5 (hold to repeat); the app always starts muted (`MUTED` in the top bar) |
| F1 | back to the launcher |

Settings (frequency, mode, gain, ppm, squelch, timeslot selection, step, memory bank, recording
format, SDS log) are kept in NVS. Letter keys work in either case. The bottom line of the screen names the
keys that matter at the moment.

### Settings

F2 opens the settings in the lower part of the screen, in three columns; the spectrum stays
visible and follows every change at once. UP / DOWN select, LEFT / RIGHT or `+` / `-` change, F2
or ESC closes.

| Setting | Values |
|---|---|
| Freq. corr. | -200 … +200 ppm, the error of the dongle's crystal |
| RF gain | AUTO, then the gain steps of the tuner |
| Squelch | off, -100 … 0 dBFS |
| Step | tuning step, 10 Hz … 1 MHz |
| Span | of the spectrum: wide / narrow |
| Mode | NFM … TETRA |
| DMR slot, TETRA slot | AUTO or one timeslot (see [Timeslots](#timeslots)) |
| Mem. bank | 0 … 9 |
| Rec. format | Opus 16k, Opus 32k, WAV ADPCM (see [Recording](#recording)) |
| SDS log | off / to SD card: write the received [messages](#messages) to a file |

### The frequency mark

The red line in the middle of the spectrum is the tuned frequency. The amber band around it, with
its bar on the frequency scale and the two lines down the waterfall, is the part of the spectrum
the current mode demodulates: 15 kHz in NFM, 10 kHz in AM, 12.5 kHz in DMR, 25 kHz in TETRA,
200 kHz in WFM. In LSB and USB the red line is the suppressed carrier and the band is the audio
pass band beside it, 0.3 to 2.7 kHz below or above. A signal is tuned in when it fills the band.

### Squelch

SPACE switches the squelch on and off. It comes back at the level it had; the first time it is set
6 dB above what the channel carries at that moment, so press it while the channel is quiet.
To set the level, press `S`: the bottom line shows the level and whether the squelch is open, and
`+` / `-` move it in 1 dB steps (switching the squelch on if it was off). `S` again, or ESC, ends
this and `+` / `-` are the gain keys again. The level is also in the settings (F2), where stepping
down past -100 dBFS switches the squelch off. The digital modes have no squelch.

### Timeslots

Only one timeslot goes to the speaker. With the selection on AUTO that is the first one that
carries clear speech; it keeps the speaker until it has been quiet for a moment (0.2 s in DMR,
0.4 s in TETRA), then the next one to talk gets it. SPACE steps through AUTO and the timeslots
(`TS1`, `TS2` in DMR; `TN1` … `TN4` in TETRA): with a timeslot selected, only that one is heard,
whatever the others do. The panel shows the selection (`audio: AUTO`), marks the slot that is
being heard (`VOICE (audio)` in DMR, `AUDIO` in TETRA) and, in DMR, the one that is not
(`VOICE (muted)`). All timeslots are decoded and shown either way. The selection is kept per mode.
For DMR signals that come from a mobile rather than a repeater or hotspot the real slot number
cannot be known; the two slots are then numbered in order of appearance.

### Memories

Ten banks of ten memories, each a frequency and a mode. After F3 (or `M`) the bottom of the screen lists
the current bank; ESC leaves without doing anything, and so does waiting eight seconds. The
memories live in a text file on the internal FAT partition, `/int/sdr-dv/memories.txt` (the `/int`
of the launcher's file browser and of BadgeLink), which can be edited on a PC:

```
# bank memory MHz mode
0 1 391.262500 TETRA
0 4 430.400000 DMR
2 7 7.100000 LSB
```

The file is read when the app starts and rewritten on every save; lines that do not parse are
dropped at that point, comments included. On the first start it is created with a few examples.

### Recording

F6 records what the receiver demodulates to a file on the SD card, F6 again stops. A red circle
with `REC` and the recorded time shows in the top bar, and [LED B](#leds) turns red. The file is

```
/tanmatsu-sdr-dv/yyyymmdd-hhmmss-ffffff-mode.opus      20261116-122144-430400-dmr.opus
/tanmatsu-sdr-dv/MCC-MNC/yyyymmdd-...-tetra.opus       214-0007/20261116-122144-391413-tetra.opus
```

with the local time (the clock and time zone of the launcher), the frequency in kHz and the mode,
all as they were when the recording started. A TETRA recording goes into a folder named after the
network of the cell, mobile country code and mobile network code (see
[Files on the SD card](#files-on-the-sd-card)); if the cell was not decoded yet when the recording
started, the file is moved there when the recording ends. The format is a setting (F2):

| Format | What | Size |
|---|---|---|
| Opus 16k | Ogg Opus, 16 kHz, 16 kbit/s, tuned for speech | 7.5 MB per hour |
| Opus 32k | Ogg Opus, 24 kHz, 32 kbit/s, for broadcast FM and music | 15 MB per hour |
| WAV ADPCM | IMA ADPCM, 8 kHz, 4 bit; plays on anything | 14.5 MB per hour |

Only audio that is there is recorded: the analogue modes while the squelch is open (always, with
the squelch off), DMR and TETRA while there is speech on the timeslot that is heard. A recording
left running therefore holds the transmissions back to back and not the silence in between, and
its length is the length of the audio, not of the wait. A recording in which nothing was heard
leaves no file. The volume setting does not matter; a muted receiver records as well.

The card is mounted when a recording starts and released when it ends, and every five seconds the
file is brought to a state that plays, so a flat battery or a pulled card costs the last seconds
at most. The card has to be FAT16 or FAT32: exFAT, which cards of 64 GB and more come with, is not
supported, and the bottom line says so.

The encoder is libopus 1.5.2 in its fixed-point build (the P4 has no faster one: Espressif's
optimised Opus is binary-only, which does not go with this app's licence). It shares a core with
the display, so the display slows from 30 frames per second to 20 – 25 while an Opus recording
runs, 27 with ADPCM. The receiver, the decoders and the audio are on higher priorities and are
not affected: no samples were lost in any of the measurements.

### Messages

TETRA text messages (SDS), status messages and other short data that the cell sends in the clear,
and DMR text and status messages, are kept in a list, the last 32 of them. F5 shows it in place of the spectrum, newest first: time,
frequency, sender and addressee, and the text, on up to four lines each. UP / DOWN scroll; F5, ESC
or RETURN close. Short data that is not text is shown as its type, protocol identifier and the
bits in hex. A message that the network repeats within ten seconds is listed once. The list is
gone when the app exits.

On DMR a message is a data packet: a header and up to some dozen data blocks, coded at rate ½, ¾
or 1, confirmed (every block with its own checksum) or not. The packet is put together, checked
with its CRC-32 and read as one of the text formats in use: Motorola TMS, Hytera TMP, the short
data formats of the DMR standard (ISO 8859, UTF-8, UTF-16, 7 bit), or plain UTF-16 / UTF-8 text
that is recognised as such. The sender is the radio ID, the addressee the radio ID or `TG` and
the talk group. A text from a packet that lost blocks or failed its checksum is listed with
`[incomplete]` or `[CRC error]` in front. Packets of unknown content are listed in hex with their
service access point and UDP ports; registration (ARS), location (LRRP), telemetry and
acknowledgements only appear as a line in the event log of the decoder panel.

With the setting *SDS log* (F2) on, every message is also appended to a text file on the SD card,
one file per day and network:

```
/tanmatsu-sdr-dv/MCC-MNC/yyyymmdd-sds.txt        214-0007/20261116-sds.txt
```

```
# date	time	MHz	system	network	from	to	kind	content
2026-11-16	15:14:02	391.012500	TETRA	214-0007	1234567	7654321	text	Meeting at 10
2026-11-16	15:14:40	391.012500	TETRA	214-0007	1234567	7654321	status	32768
2026-11-16	15:20:11	430.400000	DMR	-	2141234	TG 91	text	Hello all
```

DMR has no network identity: its messages go to the file in the top folder, with `-` as network.

The fields are separated by tabs; `kind` is `text`, `status` or `data`. Tabs, line ends and
backslashes inside a field are written as `\t`, `\n`, `\r`, `\\`, other control characters as
`\xNN`, so a line is always one message. The card is used only while a message is written and can
be taken out in between. If it cannot be written, the bottom line says so once and the list on the
screen carries on.

The header of the F5 page shows whether the log is on, how many messages were written and to
which file.

### Screenshots

Fn+F6 saves what is on the screen as a PNG, 800 x 480, 50 to 250 KB depending on how much
waterfall is in the picture:

```
/tanmatsu-sdr-dv/yyyymmdd-hhmmss-ffffff-mode.png
```

It takes a second or two, the receiver carries on, and the bottom line shows the file name when it
is done.

### Files on the SD card

Everything the app writes to the card is in the folder `tanmatsu-sdr-dv`:

```
tanmatsu-sdr-dv/
  20261116-122144-430400-dmr.opus        recordings of the analogue modes and of DMR
  20261116-151745-391013-tetra.png       screenshots
  20261116-sds.txt                       DMR messages, TETRA ones from before the cell was identified
  214-0007/                              one folder per TETRA network: MCC-MNC
    20261116-122144-391413-tetra.opus    TETRA recordings
    20261116-sds.txt                     the messages of the day
```

The network is the one the cell broadcasts: three digits of country code, four of network code.
DMR has no such identity, so its recordings stay in the top folder. Earlier versions wrote to
`sdr-dv`; that folder is renamed to `tanmatsu-sdr-dv` the first time the app writes to the card,
unless a `tanmatsu-sdr-dv` is there already.

### LEDs

The app uses the two LEDs marked A and B. The power LED and the other three stay with the
Tanmatsu's keyboard controller, which shows battery and charging on them as it does everywhere
else.

| LED | shows |
|---|---|
| A | the signal in the tuned channel over the noise floor: off below 3 dB, red for a weak signal, through yellow to green at 30 dB and more |
| B green | analogue audio is playing (NFM, AM, LSB, USB, WFM): the squelch is open, or switched off |
| B blue | digital audio is playing: DMR / TETRA speech is being decoded |
| B red | a recording is running |

The colours of B mix: yellow is a recording of analogue audio, magenta a recording of digital
speech. B shows what the receiver puts out, whatever the volume: it lights with the app muted too.

Brightness follows the LED brightness of the Tanmatsu (launcher, Settings, Brightness) up to 10 %
and stops there; at the launcher's default of 100 % A and B are therefore much dimmer than the
power LED. The LEDs are looked at four times a second and only written when a colour changes.

### Below 24 MHz: direct sampling

Tuning below 24.3 MHz switches the dongle to direct sampling: the tuner is bypassed and the RTL2832
samples the antenna input itself (Q branch). The top bar shows `direct` in place of the RF gain.
There is no tuner gain in this mode, so `+` / `-` do nothing and `G` switches the RTL2832's digital
AGC on and off. Going back up restores the tuner and its gain. The lowest frequency is
312 kHz. This needs a dongle that the driver can switch (see [Limits](#limits)); on any other the
status line says that the dongle cannot tune there.

The small window at the bottom right is a scope of the demodulated signal. In TETRA mode it is a
constellation: one dot per symbol, as the receiver decides on it (the phase step from the symbol
before), with marks where a clean π/4-DQPSK signal lands. Four tight groups on the marks mean a good
signal; a pattern turned off the marks is a carrier offset not yet corrected; an even cloud is
noise or no TETRA carrier.

Two things that matter for the digital modes:

- **Frequency correction.** A TETRA carrier has to be within about a kHz, so set the ppm value of
  your dongle in the settings (F2). The TETRA panel shows the remaining carrier offset in Hz; the DMR and
  TETRA panels both show the measured clock error in ppm, which is the same number.
- **Gain.** A nearby hotspot overloads the dongle at the default gain. The status line shows the
  ADC range and turns to a clip warning; lower the gain until it goes away.

## Build and install

Needs ESP-IDF v6.0 and the Tanmatsu SDK set-up of the
[template app](https://github.com/Nicolai-Electronics/tanmatsu-template-app).

```sh
make build                 # -> build/tanmatsu/application.bin
make badgelink             # once: fetches the BadgeLink tools
make install && make run   # device in USB device mode (launcher: purple diamond key)
```

The app is an AppFS application. Do not flash it as the main firmware.

`make install` also copies `metadata/` (`metadata.json`, `icon32.png`, `icon64.png`) to
`/int/apps/nl.tanmatsu.sdr-dv/` on the device, which is where the launcher takes the name and the
icon of the app from. The icons are made from `icon.png` with `make appicon` (ImageMagick).

### TETRA voice

The TETRA speech codec is not part of this repository: it is ETSI's reference code, which may not
be redistributed. To build with voice, download `en_30039502v010301p0.zip` (EN 300 395-2 V1.3.1)
from etsi.org, put it in the project root and build; the build log says
`tetra_codec: TETRA voice built in`. Without the file everything else works and a call is shown
but not played. **A binary built with the codec is for your own device only — do not pass it on.**
Details and the tests in [`components/tetra_codec`](components/tetra_codec/README.md).

## How it works

```
RTL2832U 1.152 MS/s, 8 bit IQ, tuned 288 kHz below the channel
  -> fs/4 shift + FIR, /8            144 kHz    (narrow spectrum, WFM)
  -> FIR, /3                          48 kHz    NFM, AM, DMR (FM discriminator -> dmr_rx)
       -> shift, FIR, /4              12 kHz    LSB, USB (sideband filter, AGC, back up to 48 kHz)
  -> FIR, /2                          72 kHz    TETRA (4 samples per symbol -> tetra_rx)
```

- `main/` — board bring-up and keys (`main.c`), USB / tuner control and the DSP task (`radio.c`,
  `dsp.c`), audio output (`audio.c`), display (`ui.c`), the glue between the decoders and the
  screen (`decoder.c`), the recorder (`rec.c`: SD card, file, task; `recfmt.c`: resampler, Opus
  in Ogg, ADPCM in WAV), the message list and its file (`msglog.c`), screenshots (`shot.c`: PNG
  with the deflate coder in the P4's ROM), the LEDs (`leds.c`), development console (`console.c`,
  `capture.c`).
- `components/dmr_rx/` — DMR receiver library: 4FSK demodulator, sync search, BPTC / Golay /
  Hamming / RS / trellis FEC, link control, CSBK, talker alias, AMBE+2 decoder. Portable, with
  host tests; see its README.
- `components/tetra_rx/` — TETRA downlink receiver library: π/4-DQPSK demodulator, burst
  synchronisation, lower MAC (descrambling, de-interleaving, Viterbi, CRC), upper MAC, LLC, MLE /
  CMCE / MM parsing. Based on [osmo-tetra](https://osmocom.org/projects/tetra), reworked for a
  small fixed memory footprint and single-precision arithmetic. Portable, with host tests; see its
  README.
- `components/tetra_codec/` — wrapper that builds the ETSI reference speech decoder from the
  archive, if present (see above).
- `components/esp_rtl_sdr/` — the RTL2832U USB host driver, v0.9.6, unmodified.
- `components/opus/` — the encoder half of libopus 1.5.2, fixed point (see its `CMakeLists.txt`
  for the one change against upstream).

Measured on the Tanmatsu (ESP32-P4 at 360 MHz) with the display running at 30 fps: the DSP core is
about 60 % busy in TETRA mode (65 % while a voice call is decoded) and 70 % in DMR mode during a
call, of which the decoder itself takes a quarter to a third of the core; SSB takes about 40 %.
No samples are dropped. Both decoders keep their state in internal RAM (about 50 KB of it stays
free); PSRAM holds only the audio buffer. Encoding a recording takes 5.7 ms of the other core
per 20 ms of audio for Opus 16k, 3.2 ms for Opus 32k, at encoder complexity 3.

In the SSB modes the dongle is only retuned when the wanted frequency is more than 5 kHz from where
it stands; in between, the 10 Hz steps are done in software and nothing is interrupted. The USB
driver wants six 16 KB DMA buffers from internal RAM when the dongle starts, so `radio.c` sets that
room aside before the rest of the app allocates anything and releases it just before the start.

## Development console

The app listens on the ESP32-P4's USB-Serial-JTAG port, so it can be driven and inspected from a PC
without touching the device. `tools/tsdr.py` is the host side:

```sh
tools/tsdr.py cmd stat                     # status, signal levels, CPU, the decoder panel
tools/tsdr.py cmd "freq 430400000"         # also: mode, gain, ppm, sq, zoom, vol, key, exit
tools/tsdr.py cmd declog                   # the decoder's event log (16 entries)
tools/tsdr.py cmd mem                      # the memories and the state of the memory file
tools/tsdr.py cmd "key m4"                 # type keys: load memory 4
tools/tsdr.py cmd "key fn-tab"             # ... by name: left, tab, esc, space, f2 ... f6, with fn- or shift-
tools/tsdr.py cmd "hold right 1500"        # LEFT or RIGHT held down for 1.5 s (key repeat)
tools/tsdr.py cmd menu                     # the settings as the F2 menu shows them
tools/tsdr.py cmd "recfmt 1"               # recording format 0 .. 2
tools/tsdr.py cmd ls                       # the files on the SD card, network folders included
tools/tsdr.py get <name> -o file.opus      # fetch one; <name> as ls prints it, e.g. 214-0007/x.opus
tools/tsdr.py cmd "rm <name>"              # delete one, or a folder that is empty
tools/tsdr.py cmd msgs                     # the message list of F5
tools/tsdr.py cmd "sdslog 1"               # the SDS log setting, 0 or 1
tools/tsdr.py cmd "sdstest 214 7 Hello"    # put a made-up message in the list (MCC, MNC, text; MCC 0: no network)
tools/tsdr.py cap fm 3000 -o x.s16         # record a tap: raw | if | ch | fm | audio
tools/tsdr.py cap fm 3000 --trig -40 ...   # ... starting when the channel exceeds -40 dBFS
tools/tsdr.py shot screen.png              # screenshot over USB
tools/tsdr.py cmd "key fn-f6"              # ... or to the SD card, as the key does
```

Captures can be replayed through the decoders on the PC (`components/dmr_rx/host`,
`components/tetra_rx/host`); `testdata/` holds a few off-air recordings made this way.
`tests/test_dsp.c` checks the decimation chain on the host, `tests/test_ssb.c` the SSB demodulator
(pass band, sideband rejection, AGC), `tests/test_rec.c` writes a file in each recording format
with the code the firmware uses, to be checked with `opusinfo`, `opusdec` and `ffmpeg`.

## Limits

- Direct sampling (below 24 MHz) only with dongles for which the driver has it: RTL-SDR Blog V3 and
  Nooelec NESDR SMArt v5, on the Q branch. Whether a V4, which has an upconverter instead, tunes
  there is up to the driver; it has not been tried with this app. Without an HF antenna on the
  dongle there is little to hear.
- TETRA voice needs the ETSI codec archive at build time (see [TETRA voice](#tetra-voice)). One
  timeslot at a time goes to the speaker (see [Timeslots](#timeslots)). A speech frame whose slot
  was half stolen for signalling is concealed rather than decoded.
- TETRA: downlink of trunked mode (TMO) only. No direct mode (DMO), no uplink, and calls are not
  followed to another carrier: a call that the control channel assigns elsewhere is shown with its
  carrier and timeslot, and you tune there by hand.
- Encrypted traffic, DMR or TETRA, is detected and flagged, never decrypted.
- Recording, message log, screenshots: SD cards with FAT16 / FAT32 only. The file name and the
  tags of a recording keep the frequency and mode it started with, also if you tune elsewhere
  while it runs.
- Messages: only those sent in the clear on the frequency that is tuned. Text in the Latin
  alphabet is shown as sent; characters the display font does not have appear as `?` in the list
  (the file has them, in UTF-8).
- DMR messages: encrypted ones are not readable, and nothing in the packet says so: they end up
  in the list as hex data. Formats other than the ones named under [Messages](#messages) do too.
  A text that is not in one of those formats is found by looking for a run of UTF-16 or UTF-8
  characters, which can leave a stray character in front of it. At rate ¾ the block coding lets
  some errors through; the checksums catch them, so such a message is marked rather than wrong
  without notice, but an unconfirmed packet cannot say which block it was. The Motorola, Hytera
  and short data formats were written from public descriptions and tested with generated signals
  and one recorded acknowledgement, not yet with every radio.
- DMR: voice only for AMBE+2 (the standard vocoder). One timeslot at a time goes to the speaker.
  Of the data services only text and status messages are read; IP data is shown as what it is
  (ports, length), not interpreted.

## Licence

AGPL-3.0 (see `LICENSE`). This follows from the parts it is built on:

- `esp_rtl_sdr` — AGPL-3.0-only.
- `tetra_rx` — AGPL-3.0-or-later, derived from osmo-tetra (AGPL-3.0-or-later).
- `dmr_rx` — AGPL-3.0-only, with FEC code from MMDVMHost (GPL-2.0-or-later) and the vocoder from
  mbelib / dsd (ISC) in `third_party/`.
- `opus` — libopus, BSD 3-clause (`components/opus/libopus/COPYING`).

The TETRA speech codec, when built in, is ETSI's copyrighted reference code and is not covered by
the licence above; see `components/tetra_codec/README.md`.

The AMBE+2 vocoder is covered by patents in some countries; the mbelib-derived decoder here is
meant for education and experiments, and whether you may use it is for you to check. Listening to
radio traffic that is not meant for you is restricted in many places as well.
