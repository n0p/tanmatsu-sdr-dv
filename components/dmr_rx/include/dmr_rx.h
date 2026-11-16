/*
 * dmr_rx - DMR (ETSI TS 102 361) receive / decoder library.
 *
 * Input : 48 kHz real FM-discriminator samples (arbitrary gain, DC offset and
 *         polarity).
 * Output: synchronous event callback + pollable status.
 *
 * SPDX-License-Identifier: AGPL-3.0-only
 * (contains GPL-2.0-or-later and ISC licensed third party code, see
 *  third_party/ and README.md)
 */
#ifndef DMR_RX_H
#define DMR_RX_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** The only supported input sample rate. */
#define DMR_RX_SAMPLE_RATE 48000

/** PCM produced by the optional vocoder (DMR_RX_WITH_VOICE). */
#define DMR_RX_PCM_RATE          8000
#define DMR_RX_PCM_FRAME_SAMPLES 160 /* one 20 ms AMBE+2 frame */

typedef struct dmr_rx dmr_rx_t;

/* ------------------------------------------------------------------------ */
/* Enumerations                                                             */
/* ------------------------------------------------------------------------ */

/** Sync pattern of a burst (TS 102 361-1 table 9.2). */
typedef enum {
    DMR_SYNC_NONE = 0,
    DMR_SYNC_BS_VOICE,     /* repeater downlink, voice superframe burst A */
    DMR_SYNC_BS_DATA,      /* repeater downlink, data / control burst     */
    DMR_SYNC_MS_VOICE,     /* MS sourced (uplink or simplex), voice       */
    DMR_SYNC_MS_DATA,      /* MS sourced (uplink or simplex), data        */
    DMR_SYNC_DM_TS1_VOICE, /* TDMA direct mode, timeslot 1, voice         */
    DMR_SYNC_DM_TS1_DATA,  /* TDMA direct mode, timeslot 1, data          */
    DMR_SYNC_DM_TS2_VOICE, /* TDMA direct mode, timeslot 2, voice         */
    DMR_SYNC_DM_TS2_DATA,  /* TDMA direct mode, timeslot 2, data          */
    DMR_SYNC_TYPE_COUNT
} dmr_sync_type_t;

/** Slot Type "data type" field (TS 102 361-1 table 6.1). */
typedef enum {
    DMR_DT_PI_HEADER        = 0,
    DMR_DT_VOICE_LC_HEADER  = 1,
    DMR_DT_TERMINATOR_LC    = 2,
    DMR_DT_CSBK             = 3,
    DMR_DT_MBC_HEADER       = 4,
    DMR_DT_MBC_CONTINUATION = 5,
    DMR_DT_DATA_HEADER      = 6,
    DMR_DT_RATE_12_DATA     = 7,
    DMR_DT_RATE_34_DATA     = 8,
    DMR_DT_IDLE             = 9,
    DMR_DT_RATE_1_DATA      = 10,
    DMR_DT_UNIFIED_SINGLE   = 11
    /* 12..15 reserved */
} dmr_data_type_t;

/** Full Link Control opcodes that the library interprets. */
typedef enum {
    DMR_FLCO_GROUP_VOICE   = 0x00,
    DMR_FLCO_UNIT_TO_UNIT  = 0x03,
    DMR_FLCO_TALKER_ALIAS0 = 0x04, /* header */
    DMR_FLCO_TALKER_ALIAS1 = 0x05,
    DMR_FLCO_TALKER_ALIAS2 = 0x06,
    DMR_FLCO_TALKER_ALIAS3 = 0x07,
    DMR_FLCO_GPS_INFO      = 0x08,
    DMR_FLCO_TERMINATOR_DATA = 0x30
} dmr_flco_t;

/** Service options octet of a voice LC (TS 102 361-2 7.2.1). */
#define DMR_SVC_EMERGENCY     0x80u
#define DMR_SVC_PRIVACY       0x40u /* call is encrypted                    */
#define DMR_SVC_BROADCAST     0x08u
#define DMR_SVC_OVCM          0x04u
#define DMR_SVC_PRIORITY_MASK 0x03u

/** Where a Link Control PDU came from. */
typedef enum {
    DMR_LC_SRC_VOICE_HEADER = 0, /* Voice LC Header burst, BPTC + RS(12,9)  */
    DMR_LC_SRC_TERMINATOR   = 1, /* Terminator with LC burst                */
    DMR_LC_SRC_EMBEDDED     = 2  /* reassembled from voice bursts B..E      */
} dmr_lc_source_t;

typedef enum {
    DMR_EVT_SYNC_ACQUIRED = 1, /* a timeslot has been acquired              */
    DMR_EVT_SYNC_LOST,         /* a timeslot has been lost                  */
    DMR_EVT_DATA_BURST,        /* every data-sync burst (slot type info)    */
    DMR_EVT_VOICE_BURST,       /* every voice burst: 3 AMBE+2 frames        */
    DMR_EVT_LC,                /* a CRC/RS-valid full link control          */
    DMR_EVT_CSBK,              /* a CRC-valid CSBK (or MBC header)          */
    DMR_EVT_PI_HEADER,         /* a CRC-valid privacy indicator header      */
    DMR_EVT_DATA_HEADER,       /* a CRC-valid data header                   */
    DMR_EVT_SHORT_LC,          /* a CRC-valid short LC from the CACH        */
    DMR_EVT_TALKER_ALIAS,      /* talker alias text updated                 */
    DMR_EVT_CALL_START,        /* voice call became active on a slot        */
    DMR_EVT_CALL_END,          /* voice call ended (terminator or loss)     */
    DMR_EVT_CALL_UPDATE,       /* call info changed (late entry ids known,
                                  encryption detected); payload u.call      */
    DMR_EVT_DATA_PDU           /* a data packet (header + data blocks) is
                                  over: text message, status, ...           */
} dmr_event_type_t;

/** Polarity handling of the discriminator signal. */
typedef enum {
    DMR_POLARITY_AUTO = 0,
    DMR_POLARITY_NORMAL,   /* positive sample = positive frequency deviation */
    DMR_POLARITY_INVERTED
} dmr_polarity_t;

/* ------------------------------------------------------------------------ */
/* Event payloads                                                           */
/* ------------------------------------------------------------------------ */

/** Signal quality figures of one burst. */
typedef struct {
    uint8_t sync_errs;   /* bit errors in the 48 bit sync (0xFF: burst had no
                            sync word or it was not found)                   */
    uint8_t fec_errs;    /* bits corrected by the FEC that could be checked
                            in this burst                                    */
    uint16_t fec_bits;   /* number of channel bits those FECs covered        */
    float   evm;         /* RMS symbol error relative to the outer deviation
                            (0 = perfect, ~0.17 = FEC starts to struggle)    */
} dmr_burst_quality_t;

typedef struct {
    uint8_t data_type;   /* dmr_data_type_t                                  */
    uint8_t slot_type_ok;/* Golay(20,8) decoded                              */
    uint8_t payload_ok;  /* payload plausible / its CRC ok (rate 1 data
                            is not protected at all: always 1)               */
    uint8_t payload_len; /* valid bytes in payload[]: 12 (BPTC(196,96)),
                            18 (rate 3/4 data), 24 (rate 1 data) or 0        */
    uint8_t payload[24]; /* the decoded info bits                            */
} dmr_evt_data_burst_t;

typedef struct {
    uint8_t burst_index;    /* 0 = A (sync burst) ... 5 = F                  */
    uint8_t encrypted;      /* privacy bit / PI header seen for this call    */
    uint8_t emb_ok;         /* bursts B..F: EMB QR(16,7,6) decoded           */
    uint8_t lcss;           /* bursts B..F: LCSS from the EMB                */
    /* The three 72 bit AMBE+2 frames exactly as transmitted (MSB first).
       This is the format used by DVSI hardware and MMDVM.                  */
    uint8_t frame[3][9];
    /* The same frames deinterleaved, Golay-corrected and descrambled into
       the 49 voice parameter bits (MSB first, bit 0 of the vocoder first;
       same order as mbelib's ambe_d[] and DSD's .amb files).               */
    uint8_t ambe49[3][7];
    uint8_t errs[3];        /* per frame: Hamming distance of the received
                               C0 (24 bit) and C1 (23 bit) Golay code words to
                               the decoded ones, 0..7. 0..3 is a good frame,
                               noise gives 5..7                              */
    uint8_t bad[3];         /* do not use the frame (conceal it): C0 is not
                               correctable, or nothing recognisable was
                               received at the position of this burst (fade,
                               transmission ended without terminator)        */
} dmr_evt_voice_burst_t;

typedef struct {
    uint8_t source;         /* dmr_lc_source_t                               */
    uint8_t pf;             /* protect flag                                  */
    uint8_t flco;           /* 6 bit FLCO (dmr_flco_t for the known ones)    */
    uint8_t fid;            /* feature set id (0 = ETSI standard)            */
    uint8_t service_options;/* DMR_SVC_* (group / unit-to-unit voice only)   */
    uint8_t is_group;       /* flco == group voice channel user              */
    uint8_t encrypted;      /* service_options & DMR_SVC_PRIVACY             */
    uint8_t emergency;      /* service_options & DMR_SVC_EMERGENCY           */
    uint32_t dst_id;        /* talkgroup or unit id                          */
    uint32_t src_id;
    uint8_t raw[9];         /* the 72 LC bits                                */
} dmr_evt_lc_t;

typedef struct {
    uint8_t last_block;     /* LB                                            */
    uint8_t pf;
    uint8_t opcode;         /* CSBKO, 6 bits                                 */
    uint8_t fid;
    uint8_t is_mbc;         /* came from an MBC header instead of a CSBK     */
    uint8_t has_ids;        /* dst_id / src_id below are meaningful          */
    uint32_t dst_id;        /* bytes 4..6, for the opcodes where the common
                               "target, source" layout applies              */
    uint32_t src_id;        /* bytes 7..9                                    */
    uint8_t raw[10];        /* opcode byte, FID and the 8 payload octets     */
    const char *name;       /* static string, never NULL                     */
} dmr_evt_csbk_t;

typedef struct {
    uint8_t algorithm_id;
    uint8_t fid;
    uint8_t key_id;
    uint32_t mi;            /* initialisation vector                         */
    uint32_t dst_id;
    uint8_t raw[10];
} dmr_evt_pi_header_t;

typedef struct {
    uint8_t is_group;
    uint8_t response_requested;
    uint8_t dpf;            /* data packet format, 4 bits                    */
    uint8_t sap;            /* service access point, 4 bits                  */
    uint8_t blocks_to_follow;
    uint32_t dst_id;
    uint32_t src_id;
    uint8_t raw[10];
} dmr_evt_data_header_t;

/** What the content of a data packet was recognised as. */
typedef enum {
    DMR_PDU_UNKNOWN = 0,    /* not recognised: data / len is all there is    */
    DMR_PDU_TEXT,           /* a text message: text                          */
    DMR_PDU_STATUS,         /* a status / precoded message: status           */
    DMR_PDU_SERVICE         /* a known service that carries no text (location,
                               registration, acknowledgements): format      */
} dmr_pdu_kind_t;

/** A data packet: the header(s) and the data blocks that followed, put
 *  together. Sent when the last announced block has passed, or earlier when
 *  the transmission ends or turns to something else (complete == 0 then). */
typedef struct {
    uint8_t is_group;
    uint8_t response_requested;
    uint8_t dpf;            /* data packet format of the (first) header      */
    uint8_t sap;            /* service access point of the content (from the
                               proprietary header if there was one)          */
    uint8_t confirmed;      /* the blocks carry a serial number and a CRC-9  */
    uint8_t block_type;     /* dmr_data_type_t of the data blocks, 0xFF if
                               there were none                               */
    uint8_t blocks;         /* blocks announced by the header, the proprietary
                               headers included                              */
    uint8_t blocks_missing; /* of those: not received                        */
    uint8_t complete;       /* nothing is missing                            */
    uint8_t crc_ok;         /* the CRC-32 over the packet is right (only a
                               complete packet can be checked)               */
    uint8_t prop_headers;   /* number of proprietary headers                 */
    uint8_t mfid;           /* manufacturer id from the first of them        */
    uint8_t kind;           /* dmr_pdu_kind_t                                */
    uint16_t status;        /* DMR_PDU_STATUS: the 10 bit status value       */
    uint16_t src_port;      /* UDP ports if the packet is UDP/IP, else 0     */
    uint16_t dst_port;
    uint32_t dst_id;
    uint32_t src_id;
    uint16_t len;           /* bytes in data                                 */
    /* The user data of the packet without block serial numbers / CRC-9 and,
       if the packet is complete, without pad octets and CRC-32. Blocks that
       were not received are zeros. Only valid during the callback.          */
    const uint8_t *data;
    const char *format;     /* what the content is, static string, never NULL */
    /* DMR_PDU_TEXT: the message, UTF-8, lines separated by '\n'; "" for the
       other kinds. Only valid during the callback.                          */
    const char *text;
    uint8_t header[10];     /* the data header without its CRC               */
    uint8_t prop_header[10];/* the first proprietary header, if any          */
} dmr_evt_data_pdu_t;

typedef struct {
    uint8_t slco;           /* short LC opcode, 4 bits                       */
    uint8_t data[3];        /* 24 payload bits                               */
    const char *name;       /* static string, never NULL                     */
} dmr_evt_short_lc_t;

typedef struct {
    uint8_t complete;       /* all announced blocks received                 */
    char text[64];          /* UTF-8, NUL terminated                         */
} dmr_evt_talker_alias_t;

typedef struct {
    uint8_t group;          /* 1 group call, 0 unit-to-unit                  */
    uint8_t encrypted;
    uint8_t emergency;
    uint8_t late_entry;     /* call picked up from embedded LC / voice sync  */
    uint8_t ids_valid;      /* dst_id / src_id known                         */
    uint32_t dst_id;
    uint32_t src_id;
    uint32_t voice_bursts;  /* CALL_END: number of voice bursts received
                               (without the ones that were only noise)       */
} dmr_evt_call_t;

typedef struct {
    uint8_t type;           /* dmr_event_type_t                              */
    /* TDMA timeslot (1 or 2). For repeater downlinks this is taken from the
       CACH TACT, for TDMA direct mode from the sync pattern. For MS sourced
       bursts the real timeslot number cannot be known; the library then
       labels the two timing phases 1 and 2 in order of appearance and
       clears slot_known.                                                   */
    uint8_t slot;
    uint8_t slot_known;
    uint8_t sync_type;      /* dmr_sync_type_t of the last sync on this slot */
    uint8_t color_code;     /* 0..15, 0xFF if not yet known                  */
    uint8_t inverted;       /* signal polarity in use                        */
    uint64_t sample_index;  /* input sample index of the burst centre        */
    dmr_burst_quality_t quality;
    union {
        dmr_evt_data_burst_t   data;
        dmr_evt_voice_burst_t  voice;
        dmr_evt_lc_t           lc;
        dmr_evt_csbk_t         csbk;
        dmr_evt_pi_header_t    pi;
        dmr_evt_data_header_t  data_header;
        dmr_evt_data_pdu_t     data_pdu;
        dmr_evt_short_lc_t     short_lc;
        dmr_evt_talker_alias_t talker_alias;
        dmr_evt_call_t         call;
    } u;
} dmr_event_t;

/** Called synchronously from dmr_rx_process_*(). Do not call back into the
 *  same dmr_rx_t from the callback (dmr_rx_get_status() is allowed). */
typedef void (*dmr_event_cb_t)(void *user, const dmr_event_t *evt);

/** PCM callback (only with DMR_RX_WITH_VOICE): 160 samples, 8 kHz, per AMBE
 *  frame; three calls per voice burst. Not called for encrypted calls. */
typedef void (*dmr_pcm_cb_t)(void *user, uint8_t slot, const int16_t *pcm, size_t n);

/* ------------------------------------------------------------------------ */
/* Status                                                                   */
/* ------------------------------------------------------------------------ */

typedef enum {
    DMR_SLOT_IDLE = 0,      /* nothing received on this slot                 */
    DMR_SLOT_DATA,          /* data / control / idle bursts                  */
    DMR_SLOT_VOICE          /* voice superframes                             */
} dmr_slot_state_t;

typedef struct {
    uint8_t active;         /* slot currently has sync                       */
    uint8_t slot_known;     /* see dmr_event_t.slot                          */
    uint8_t state;          /* dmr_slot_state_t                              */
    uint8_t sync_type;      /* dmr_sync_type_t of the last sync              */
    uint8_t color_code;     /* 0xFF unknown                                  */
    uint8_t last_data_type; /* dmr_data_type_t of the last data burst, 0xFF  */
    uint8_t call_active;    /* voice call in progress                        */
    uint8_t ids_valid;      /* dst/src below are valid for the current call  */
    uint8_t group;
    uint8_t encrypted;
    uint8_t emergency;
    uint8_t flco;
    uint8_t fid;
    uint8_t service_options;
    uint8_t quality;        /* 0..100, for a signal bar                      */
    uint8_t last_sync_errs; /* of 48                                         */
    uint32_t dst_id;
    uint32_t src_id;
    char talker_alias[64];  /* "" if none                                    */
    float ber;              /* smoothed FEC-derived bit error rate estimate  */
    float evm;              /* smoothed RMS symbol error (see quality)       */
    uint32_t voice_bursts;  /* totals since create/reset                     */
    uint32_t data_bursts;
    uint32_t missed_bursts; /* expected bursts that could not be validated   */
    uint32_t fec_errs;      /* total corrected bits                          */
    uint32_t fec_bits;      /* total bits covered by checked FEC             */
} dmr_slot_status_t;

typedef struct {
    uint8_t synced;         /* any slot active                               */
    uint8_t inverted;       /* polarity currently assumed                    */
    uint8_t polarity_locked;/* polarity has been confirmed by FEC            */
    uint8_t reserved;
    float dc_offset;        /* in input sample units                         */
    float deviation;        /* outer symbol (+-1944 Hz) level, input units   */
    float clock_ppm;        /* estimated sample clock error vs. transmitter  */
    uint64_t samples;       /* input samples consumed                        */
    uint32_t sync_count;    /* sync words detected                           */
    uint32_t sync_loss_count;
    dmr_slot_status_t slot[2]; /* [0] = timeslot 1, [1] = timeslot 2         */
} dmr_rx_status_t;

/* ------------------------------------------------------------------------ */
/* API                                                                      */
/* ------------------------------------------------------------------------ */

typedef struct {
    dmr_event_cb_t event_cb;   /* may be NULL                                */
    void *event_user;
    dmr_pcm_cb_t pcm_cb;       /* may be NULL; needs DMR_RX_WITH_VOICE       */
    void *pcm_user;
    uint8_t polarity;          /* dmr_polarity_t, default AUTO               */
    uint8_t pcm_slot_mask;     /* bit0: decode TS1 audio, bit1: TS2 (def. 3) */
    uint8_t reserved[2];
    float pcm_gain;            /* linear gain applied to decoded PCM (def 1) */
    /* Optional allocator for the (single) state block; NULL = malloc/free.
       Useful to place the state in PSRAM or internal RAM explicitly.       */
    void *(*alloc)(size_t size);
    void (*dealloc)(void *ptr);
} dmr_rx_config_t;

/** Fill cfg with defaults. */
void dmr_rx_config_default(dmr_rx_config_t *cfg);

/** Size in bytes of the single allocation dmr_rx_create() performs. */
size_t dmr_rx_state_size(void);

/** Non-zero if the library was built with the vocoder (DMR_RX_WITH_VOICE). */
int dmr_rx_has_voice(void);

/** Create a receiver. All memory is allocated here; the processing path
 *  never allocates. Returns NULL on allocation failure. cfg may be NULL. */
dmr_rx_t *dmr_rx_create(const dmr_rx_config_t *cfg);
void dmr_rx_destroy(dmr_rx_t *rx);

/** Drop all sync / call state (e.g. after retuning). Keeps configuration.
 *  Must be called from the processing task. */
void dmr_rx_reset(dmr_rx_t *rx);

/** Feed 48 kHz FM discriminator samples. Any block size. */
void dmr_rx_process_fm(dmr_rx_t *rx, const float *samples, size_t n);
/** Same, 16 bit samples (WAV files). */
void dmr_rx_process_fm_s16(dmr_rx_t *rx, const int16_t *samples, size_t n);

/** Copy the current status. May be called from another task than the one
 *  calling dmr_rx_process_*() (lock-free snapshot, updated once per
 *  process call). */
void dmr_rx_get_status(const dmr_rx_t *rx, dmr_rx_status_t *out);

/** Select which slots are run through the vocoder (bit0 TS1, bit1 TS2).
 *  Call from the processing task. */
void dmr_rx_set_pcm_slot_mask(dmr_rx_t *rx, uint8_t mask);

/* Static name helpers (never return NULL). */
const char *dmr_rx_sync_type_name(uint8_t sync_type);
const char *dmr_rx_data_type_name(uint8_t data_type);
const char *dmr_rx_flco_name(uint8_t flco, uint8_t fid);
const char *dmr_rx_csbk_name(uint8_t opcode, uint8_t fid);
const char *dmr_rx_slco_name(uint8_t slco);
const char *dmr_rx_event_name(uint8_t event_type);

/* ------------------------------------------------------------------------ */
/* Standalone vocoder access (DMR_RX_WITH_VOICE only)                       */
/* ------------------------------------------------------------------------ */

typedef struct dmr_vocoder dmr_vocoder_t;

/** Returns NULL when built without DMR_RX_WITH_VOICE. */
dmr_vocoder_t *dmr_vocoder_create(void);
void dmr_vocoder_destroy(dmr_vocoder_t *v);
void dmr_vocoder_reset(dmr_vocoder_t *v);
/** Decode one frame of 49 AMBE+2 parameter bits (dmr_evt_voice_burst_t
 *  ambe49 format) into 160 PCM samples. errs/bad as delivered with the
 *  voice burst (used for error concealment). */
void dmr_vocoder_decode(dmr_vocoder_t *v, const uint8_t ambe49[7], int errs, int bad,
                        float gain, int16_t pcm[DMR_RX_PCM_FRAME_SAMPLES]);

#ifdef __cplusplus
}
#endif

#endif /* DMR_RX_H */
