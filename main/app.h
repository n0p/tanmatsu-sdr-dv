// SPDX-License-Identifier: AGPL-3.0-only
// Shared application state for the Tanmatsu SDR receiver.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "dsp.h"

#define APP_VERSION "0.1.0"

// ---- Rates ---------------------------------------------------------------------------------------
#define RTL_RATE      1152000u  // 28.8 MHz / 25, exact
#define RTL_LO_OFFSET (RTL_RATE / 4)  // channel sits at +fs/4 above the tuner LO
#define IF_RATE_NARROW 144000u  // stage 1 /8
#define IF_RATE_WIDE   288000u  // stage 1 /4 (broadcast FM)
#define CH_RATE_48K    48000u   // NFM / AM / DMR channel rate, also the audio rate
#define CH_RATE_TETRA  72000u   // 4 samples per TETRA symbol
#define AUDIO_RATE     48000u

typedef enum {
    MODE_NFM = 0,
    MODE_AM,
    MODE_LSB,
    MODE_USB,
    MODE_WFM,
    MODE_DMR,
    MODE_TETRA,
    MODE_COUNT,
} demod_mode_t;

extern const char* const mode_names[MODE_COUNT];
// Mode by name, case-insensitive; -1 if unknown.
int mode_from_name(const char* name);

static inline bool mode_is_ssb(demod_mode_t m) {
    return m == MODE_LSB || m == MODE_USB;
}

typedef enum {
    RADIO_NO_DEVICE = 0,
    RADIO_STARTING,
    RADIO_STREAMING,
    RADIO_ERROR,
} radio_status_t;

#define GAIN_AUTO     (-1)
#define MAX_GAINS     32
#define SNAP_LEN      2048
#define SCOPE_LEN     256

// Settings: written by the UI / console tasks, read by the DSP task.
typedef struct {
    volatile uint32_t     freq_hz;  // channel centre frequency
    volatile demod_mode_t mode;
    volatile int          gain_idx;  // index into gains[], or GAIN_AUTO
    volatile int          ppm;
    volatile int          squelch_db;  // dBFS threshold for NFM/AM audio, <= -120 = open
    volatile int          volume;      // 0..100, 0 = muted (amplifier off)
    volatile bool         zoom;        // spectrum source: false = wideband raw, true = IF stream
    // Timeslot whose speech goes to the speaker: 0 = automatic (the first one that talks), or the
    // slot number (DMR 1..2, TETRA 1..4).
    volatile int          dmr_slot;
    volatile int          tetra_slot;
    volatile int          rec_format;  // rec_format_t (recfmt.h) of the next recording
    volatile bool         sds_log;     // received messages are also written to the SD card
} app_settings_t;

#define SQUELCH_OFF     (-200)
#define DMR_SLOTS       2
#define TETRA_SLOTS     4

static inline bool mode_is_digital(demod_mode_t m) {
    return m == MODE_DMR || m == MODE_TETRA;
}

// Status: written by the DSP task, read by the UI / console tasks.
typedef struct {
    volatile radio_status_t status;
    char                    status_text[64];
    char                    device_name[64];
    char                    profile_name[24];
    int                     gains[MAX_GAINS];  // tenth dB
    volatile int            gain_count;
    volatile uint32_t       effective_sps;
    volatile uint32_t       drops;       // driver-side lost blocks since start
    volatile uint32_t       cpu_permille;  // DSP task busy share
    volatile uint32_t       dec_permille;  // part of it spent in the DMR / TETRA decoder
    volatile float          chan_db;     // channel power, dBFS
    volatile uint8_t        adc_min, adc_max;  // raw ADC range over the last second
    volatile uint32_t       clip_permille;     // share of raw samples at the ADC rails
    volatile uint32_t       blocks;
    volatile bool           squelch_open;
    volatile bool           direct;      // below 24 MHz: the RTL2832 samples the antenna itself, no tuner
    volatile bool           rtl_agc;     // ... with its digital AGC on
    volatile int32_t        soft_off_hz;  // SSB: tuned frequency minus the frequency the hardware sits on
} app_status_t;

extern app_settings_t g_set;
extern app_status_t   g_stat;

// ---- Radio / DSP task (radio.c) ------------------------------------------------------------------
void radio_reserve_memory(void);  // call before the rest of the app allocates
void radio_start(void);
// The part of the spectrum a mode demodulates, in Hz relative to the tuned frequency (which is the
// suppressed carrier in the SSB modes, so their band lies to one side of it).
void mode_band_hz(demod_mode_t mode, int32_t* lo, int32_t* hi);

// Spectrum snapshots. The UI requests one, the DSP task fills it and bumps the sequence number.
typedef struct {
    volatile bool     req;
    volatile uint32_t seq;
    volatile bool     is_zoom;     // which source this snapshot came from
    volatile uint32_t rate;        // sample rate of the snapshot
    volatile uint32_t freq_hz;     // tuned frequency at capture time
    volatile int32_t  off_hz;      // where it sits in the snapshot, relative to the channel position
    uint8_t           raw[SNAP_LEN * 2];  // wideband: CU8 IQ (channel at +fs/4)
    cf32_t            iq[SNAP_LEN];       // zoom: IF stream, channel at 0 Hz
} snapshot_t;
extern snapshot_t g_snap;

// Small scope buffer for the UI widget: demodulated signal (real), or a constellation (TETRA): one
// point per symbol, scaled to an RMS radius of 1.
typedef struct {
    volatile uint32_t seq;
    volatile bool     is_iq;
    float             a[SCOPE_LEN];
    float             b[SCOPE_LEN];
} scope_t;
extern scope_t g_scope;

// ---- Capture taps (capture.c) --------------------------------------------------------------------
typedef enum {
    TAP_RAW = 0,  // CU8 IQ at RTL_RATE, channel at +fs/4
    TAP_IF,       // CS16 IQ at the stage-1 rate
    TAP_CH,       // CS16 IQ at the channel rate (48k or 72k), before the channel filter
    TAP_FM,       // S16 discriminator output at 48k (pi rad == 32767)
    TAP_AUDIO,    // S16 mono audio at 48k
    TAP_COUNT,
} tap_t;

extern const char* const tap_names[TAP_COUNT];

typedef enum { CAP_IDLE = 0, CAP_ARMED, CAP_RECORDING, CAP_DONE } cap_state_t;

typedef struct {
    volatile cap_state_t state;
    tap_t                tap;
    uint8_t*             buf;
    size_t               size;
    volatile size_t      fill;
    float                trig_db;  // start when channel power exceeds this (<= -200: immediately)
    uint32_t             rate;
    uint32_t             freq_hz;
    demod_mode_t         mode;
} capture_t;
extern capture_t g_cap;

// Console task: allocate a PSRAM buffer for `ms` of the given tap and arm the capture. Recording
// starts with the next block whose channel power is >= trig_db.
bool cap_arm(tap_t tap, uint32_t ms, float trig_db);
void cap_release(void);
const char* cap_format(tap_t tap);

// Called from the DSP task.
void cap_feed_u8(tap_t tap, const uint8_t* data, size_t bytes, uint32_t rate);
void cap_feed_cf32(tap_t tap, const cf32_t* data, int n, uint32_t rate);
void cap_feed_f32(tap_t tap, const float* data, int n, float scale, uint32_t rate);
void cap_feed_s16(tap_t tap, const int16_t* data, int n, uint32_t rate);

// ---- Audio (audio.c) -----------------------------------------------------------------------------
void audio_init(void);
void audio_push(const int16_t* mono, int n);  // 48 kHz mono, from the DSP task
void audio_apply_volume(void);                // re-reads g_set.volume (and the headphone jack)

typedef struct {
    bool     i2s_ok;      // I2S channel found and started
    int      init_err;    // esp_err_t of the first failing set-up step, 0 = none
    int      vol_err, amp_err, last_write_err;
    bool     amplifier, headphones;
    int      codec_pct;   // what went to bsp_audio_set_volume()
    uint32_t chunks;      // 10 ms chunks accepted by the I2S driver (100 per second when it runs)
    uint32_t chunks_audio;  // ... of which taken from the ring (the rest was idle silence)
    uint32_t write_errors, overflows;
    uint32_t underruns;   // the ring ran empty while playing: at the end of every transmission, and ...
    uint32_t gaps;        // ... in the middle of one (samples arrived within 120 ms of an underrun)
    uint32_t fill;        // samples waiting in the ring
    uint32_t low_water;   // smallest fill seen right before a push while playing, since the last query
} audio_stats_t;
void audio_get_stats(audio_stats_t* out);

// ---- Recorder (rec.c) ----------------------------------------------------------------------------
// The audio that goes to the speaker, written to a file on the SD card while a recording runs.
// Everything the app writes to the card lives in one folder: recordings, message logs, screenshots.
// TETRA recordings and messages go into a folder per network in it, named MCC-MNC.
#define REC_MOUNT   "/sd"
#define REC_DIRNAME "tanmatsu-sdr-dv"
#define REC_DIR     REC_MOUNT "/" REC_DIRNAME
#define REC_DIR_OLD REC_MOUNT "/sdr-dv"  // what it was called at first: renamed when found
typedef enum { REC_IDLE = 0, REC_STARTING, REC_ON, REC_STOPPING } rec_state_t;
typedef struct {
    volatile rec_state_t state;
    char                 name[64];    // file of the running or the last recording, relative to REC_DIR
    volatile uint32_t    ms;          // audio in it
    volatile uint32_t    bytes;       // on the card (refreshed every few seconds)
    volatile uint32_t    dropped;     // samples lost because the card was too slow
    volatile uint32_t    enc_us_max, enc_us_avg, stack_free;  // longest single encoder call
    volatile int64_t     started_us;
} rec_status_t;
extern rec_status_t g_rec;
void rec_init(void);
void rec_toggle(void);     // start, or stop and close the file; reports through app_notice()
void rec_stop_wait(void);  // stop and wait until the file is closed and the card released
void rec_feed(const int16_t* pcm, int n);  // 48 kHz mono, from the DSP task
// The card, for anything else that wants to read it (console). Mounted on the first acquire,
// unmounted with the last release. On failure `why` says what is wrong.
bool rec_sd_acquire(char* why, size_t why_len);
void rec_sd_release(void);
// With the card acquired: makes sure REC_DIR exists, and the folder `sub` in it unless that is
// NULL or empty. False if it cannot be created (write protected, full).
bool rec_make_dir(const char* sub);
// The folder of a TETRA network inside REC_DIR.
void rec_plmn_dir(char* out, size_t len, unsigned mcc, unsigned mnc);
// DSP task, with every piece of TETRA speech that is recorded: the network it comes from.
void rec_note_plmn(unsigned mcc, unsigned mnc);

// ---- Received messages (msglog.c) ----------------------------------------------------------------
// TETRA short data (SDS) and status messages: the last ones are kept in memory for the F5 list,
// and with the "SDS log" setting on each one is also appended to a text file on the card,
//   <REC_DIR>/<MCC-MNC>/yyyymmdd-sds.txt
#define MSG_KEEP     32
#define MSG_TEXT_MAX 600  // the longest SDS (2047 bits) as hex digits, with its description
typedef struct {
    int64_t  time;      // time_t
    uint32_t freq_hz;
    bool     plmn;      // mcc / mnc are those of the cell it was received from
    uint16_t mcc, mnc;
    bool     to_card;   // the log was on when it arrived
    char     sys[8];    // "TETRA"
    char     from[28];  // SSI, or MCC-MNC-SSI when the sender gave its network; "?" if not in the message
    char     to[16];    // SSI, or "all"
    char     kind[8];   // "text", "status", "data"
    char     text[MSG_TEXT_MAX];  // UTF-8
} msg_t;
typedef struct {
    volatile uint32_t seen;     // messages received since the app started (repeats not counted)
    volatile uint32_t dup;      // repeats of a message within a few seconds, dropped
    volatile uint32_t written;  // lines written to the card
    volatile uint32_t errors;   // messages that could not be written
    volatile bool     card;     // the log holds the card right now
    char              file[64]; // last log file written, relative to REC_DIR
} msglog_status_t;
extern msglog_status_t g_msglog;
void msglog_init(void);
// DSP task. Never waits for the card.
void msglog_put(const char* sys, bool plmn, unsigned mcc, unsigned mnc, const char* from, const char* to,
                const char* kind, const char* text);
int  msglog_count(void);              // messages kept, 0..MSG_KEEP
bool msglog_get(int back, msg_t* out);  // 0 = the newest
void msglog_stop_wait(void);          // before the app exits: write what is pending, let go of the card

// ---- Screenshots (shot.c) ------------------------------------------------------------------------
void shot_init(void);
void shot_save(void);       // Fn+F6: the next frame, as a PNG file in REC_DIR; reports through app_notice()
void shot_stop_wait(void);  // before the app exits: finish the file that is being written
// For the console: a copy of the next frame in g_shot_buf. False if the display loop does not
// deliver one. After true, shot_release() when done with the buffer.
bool shot_grab(void);
void shot_release(void);

// ---- Console (console.c) -------------------------------------------------------------------------
void console_start(void);
// Screenshot hand-off (shot.c): the UI task copies the finished frame when asked.
extern volatile bool    g_shot_req;
extern volatile bool    g_shot_ready;
extern uint8_t*         g_shot_buf;
extern size_t           g_shot_size;

// ---- Memories (memory.c) -------------------------------------------------------------------------
// 10 banks of 10 memories (frequency + mode), kept in a text file on the internal FAT partition.
#define MEM_BANKS 10
#define MEM_SLOTS 10
#define MEM_FILE  "/int/sdr-dv/memories.txt"
void memo_init(void);
bool memo_get(int bank, int slot, uint32_t* freq_hz, demod_mode_t* mode);
// Stores the memory and rewrites the file. False if the file could not be written; the memory
// is then only kept until the app exits.
bool memo_set(int bank, int slot, uint32_t freq_hz, demod_mode_t mode);
// "ok", or why the file is not usable.
const char* memo_status(void);

// ---- Main task (main.c) --------------------------------------------------------------------------
// Memory key sequence in progress: M (load) or M S (save), waiting for a digit.
typedef enum { MEM_UI_IDLE = 0, MEM_UI_LOAD, MEM_UI_SAVE } mem_ui_t;
extern volatile mem_ui_t g_mem_ui;
extern volatile int      g_mem_bank;
// Settings menu (F2): the items in the order they are shown, column after column.
typedef enum {
    MENU_PPM = 0,
    MENU_GAIN,
    MENU_SQUELCH,
    MENU_STEP,
    MENU_SPAN,
    MENU_MODE,
    MENU_DMR_SLOT,
    MENU_TETRA_SLOT,
    MENU_BANK,
    MENU_REC_FORMAT,
    MENU_SDS_LOG,
    MENU_COUNT,
} menu_item_t;
extern volatile bool g_menu_open;
extern volatile int  g_menu_sel;
void                 menu_item_text(int item, const char** label, char* value, size_t len);
// Squelch adjustment (S): + and - move the squelch level instead of the gain until S is pressed again.
extern volatile bool g_sq_adjust;
extern volatile bool g_help_open;  // F4: the list of keys covers the spectrum
extern volatile bool g_msgs_open;  // F5: ... or the list of received messages,
extern volatile int  g_msgs_scroll;  // which starts this many messages below the newest
// One-line message shown in place of the key help for a few seconds.
void                     app_notice(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
extern char              g_notice[96];
extern volatile int64_t  g_notice_until;  // esp_timer time

void settings_mark_dirty(void);
void app_exit_to_launcher(void);
// Remote key injection for testing: nav_key is a bsp_input_navigation_key_t or 0 (with the
// BSP_INPUT_MODIFIER_* bits held down with it), ascii a char or 0.
void     app_inject_key(int nav_key, uint32_t modifiers, char ascii);
// ... and a navigation key held down for a while: one call with down = true, one with false.
void     app_inject_key_edge(int nav_key, bool down);

// ---- LEDs A and B (leds.c) -----------------------------------------------------------------------
typedef struct {
    volatile uint32_t a, b;      // colours as written, 0xRRGGBB
    volatile float    snr_db;    // what A shows
    volatile int      setting;   // LED brightness of the device, percent
    volatile int      full;      // colour value used for "on"
    volatile uint32_t sends;     // I2C writes so far
    volatile int      err;       // result of the last one
} led_status_t;
extern led_status_t g_led;
void leds_init(void);    // hands the other LEDs back to the keyboard controller, A and B dark
void leds_update(void);  // every frame; does something four times a second
void leds_off(void);
extern volatile float g_ui_fps;
extern volatile int   g_step_idx;

// Tuning helpers shared by the keyboard and the console.
// Below 24 MHz only dongles with direct sampling (RTL-SDR Blog V3 and compatibles) or an HF
// upconverter (V4) can tune. The lowest channel is the driver's 24 kHz plus the LO offset.
#define FREQ_MIN_HZ (24000u + RTL_LO_OFFSET)
#define DIRECT_SAMPLING_BELOW_HZ 24000000u
#define FREQ_MAX_HZ 1766000000u
void app_set_freq(uint32_t hz);
void app_set_mode(demod_mode_t mode);
void app_set_gain_idx(int idx);
void app_set_volume(int volume);
