/* tetra_rx - TETRA (ETSI EN 300 392-2) TMO downlink receiver/decoder library
 *
 * Input:  complex baseband at 72 000 Hz (4 samples per symbol), one 25 kHz
 *         pi/4-DQPSK downlink carrier roughly centred on 0 Hz.
 * Output: events (synchronous callback from tetra_rx_process_*) and a
 *         pollable status structure.
 *
 * The channel decoder / PDU parser is derived from Osmocom osmo-tetra
 * (AGPL-3.0-or-later, see third_party/osmo-tetra); the modem is original.
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#ifndef TETRA_RX_H
#define TETRA_RX_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TETRA_RX_SAMPLE_RATE_HZ 72000
#define TETRA_RX_SYMBOL_RATE_HZ 18000

/* largest reassembled TM-SDU / TL-SDU the library keeps (bits) */
#define TETRA_RX_MAX_SDU_BITS 4096
/* speech frame size (EN 300 395-2): 137 bits per 30 ms frame */
#define TETRA_RX_SPEECH_FRAME_BITS 137
#define TETRA_RX_TEXT_MAX 256

typedef struct tetra_rx tetra_rx_t;

/* ---- time ------------------------------------------------------------- */
typedef struct {
	uint8_t valid; /* 0 until a BSCH has been decoded since the last lock */
	uint8_t tn;    /* timeslot 1..4 */
	uint8_t fn;    /* frame 1..18 */
	uint8_t mn;    /* multiframe 1..60 */
} tetra_rx_time_t;

/* ---- logical channels used for statistics ----------------------------- */
typedef enum {
	TETRA_RX_CH_BSCH = 0, /* SB1 block of the synchronisation burst */
	TETRA_RX_CH_SB2,      /* second half slot of the sync burst (BNCH / SCH/HD) */
	TETRA_RX_CH_SCH_HD,   /* half slot signalling in normal bursts (incl. BNCH) */
	TETRA_RX_CH_SCH_F,    /* full slot signalling */
	TETRA_RX_CH_STCH,     /* stolen half slots on a traffic channel */
	TETRA_RX_CH_AACH,     /* access assignment channel, RM(30,14) */
	TETRA_RX_CH_TCH_S,    /* speech traffic, per speech-frame-pair class 2 CRC */
	TETRA_RX_CH_COUNT
} tetra_rx_chan_t;

typedef struct {
	uint32_t ok;   /* blocks with good CRC (AACH: valid codeword after correction) */
	uint32_t fail; /* blocks with bad CRC (AACH: uncorrectable) */
} tetra_rx_chan_stat_t;

/* ---- events ----------------------------------------------------------- */
typedef enum {
	TETRA_RX_EV_SYNC = 1, /* burst synchronisation gained / lost */
	TETRA_RX_EV_BSCH,     /* SYNC PDU decoded (cell identity, time) */
	TETRA_RX_EV_SYSINFO,  /* SYSINFO PDU decoded (BNCH) */
	TETRA_RX_EV_AACH,     /* ACCESS-ASSIGN (only with TETRA_RX_FLAG_AACH_EVENTS) */
	TETRA_RX_EV_SIGNAL,   /* one MAC PDU / reassembled TM-SDU with upper layers */
	TETRA_RX_EV_TRAFFIC,  /* channel-decoded TCH/S slot */
	TETRA_RX_EV_BLOCK,    /* raw type-1 block (only with TETRA_RX_FLAG_BLOCK_EVENTS) */
	TETRA_RX_EV_BURST     /* demodulated slot (only with TETRA_RX_FLAG_BURST_EVENTS) */
} tetra_rx_event_type_t;

typedef struct {
	uint8_t locked;        /* 1 = burst sync acquired, 0 = lost */
	float freq_offset_hz;  /* carrier offset estimate at that moment */
} tetra_rx_ev_sync_t;

typedef struct {
	uint16_t mcc;          /* mobile country code (10 bit) */
	uint16_t mnc;          /* mobile network code (14 bit) */
	uint8_t colour_code;   /* 6 bit */
	uint8_t system_code;   /* 4 bit */
	uint8_t tn, fn, mn;    /* as broadcast */
	uint8_t sharing_mode;  /* 0 continuous, 1 carrier sharing, 2 MCCH sharing, 3 traffic carrier sharing */
	uint8_t ts_reserved_frames;
	uint8_t u_plane_dtx;
	uint8_t frame18_ext;
	uint8_t neighbour_cell_broadcast; /* D-MLE-SYNC */
	uint8_t cell_load;                /* D-MLE-SYNC "cell load CA" */
	uint8_t late_entry;               /* D-MLE-SYNC */
	uint8_t changed;       /* 1 if identity differs from the previous BSCH */
} tetra_rx_ev_bsch_t;

typedef struct {
	uint16_t main_carrier;      /* carrier number (12 bit) */
	uint8_t freq_band;          /* 4 bit, x 100 MHz */
	uint8_t freq_offset;        /* 2 bit: 0, +6.25, -6.25, +12.5 kHz */
	uint8_t duplex_spacing;     /* 3 bit index */
	uint8_t reverse_operation;
	uint32_t dl_freq_hz;        /* main carrier downlink */
	uint32_t ul_freq_hz;        /* main carrier uplink, 0 if spacing index unknown */
	uint8_t num_secondary_cch;  /* number of common secondary control channels */
	uint8_t ms_txpwr_max_cell;
	uint8_t rxlev_access_min;
	uint8_t access_parameter;
	uint8_t radio_dl_timeout;
	uint8_t cck_valid;          /* 1: cck_id valid, 0: hyperframe valid */
	uint16_t cck_id;
	uint16_t hyperframe;
	uint8_t option_field;       /* 0 even MF TS mode, 1 odd MF, 2 access code A, 3 ext. services */
	uint32_t option_value;      /* 20 bit */
	uint16_t location_area;     /* D-MLE-SYSINFO */
	uint16_t subscriber_class;
	uint16_t bs_service_details; /* 12 bit, see TETRA_RX_BSD_* */
} tetra_rx_ev_sysinfo_t;

#define TETRA_RX_BSD_REGISTRATION   (1u << 11)
#define TETRA_RX_BSD_DEREGISTRATION (1u << 10)
#define TETRA_RX_BSD_PRIORITY_CELL  (1u << 9)
#define TETRA_RX_BSD_NO_MIN_MODE    (1u << 8)
#define TETRA_RX_BSD_MIGRATION      (1u << 7)
#define TETRA_RX_BSD_SYSTEM_WIDE    (1u << 6)
#define TETRA_RX_BSD_VOICE          (1u << 5)
#define TETRA_RX_BSD_CIRCUIT_DATA   (1u << 4)
#define TETRA_RX_BSD_SNDCP          (1u << 2)
#define TETRA_RX_BSD_AIR_ENCRYPTION (1u << 1)
#define TETRA_RX_BSD_ADVANCED_LINK  (1u << 0)

/* downlink usage marker values (ACCESS-ASSIGN) */
#define TETRA_RX_DL_UNALLOCATED  0
#define TETRA_RX_DL_ASSIGNED_CCH 1
#define TETRA_RX_DL_COMMON_CCH   2
#define TETRA_RX_DL_RESERVED     3
/* >= 4: traffic, value is the traffic usage marker */

typedef struct {
	uint8_t header;        /* 2 bit ACCESS-ASSIGN header */
	uint8_t frame18;       /* 1: frame 18 layout */
	uint8_t dl_usage_valid;
	uint8_t dl_usage;      /* see TETRA_RX_DL_* */
	uint8_t ul_usage_valid;
	uint8_t ul_usage;      /* 0 unallocated, >= 4 traffic usage marker */
	uint8_t access_valid[2];
	uint8_t access_code[2];     /* 0..3 = A..D */
	uint8_t base_frame_len[2];  /* 4 bit code */
	uint8_t corrected_bits;     /* bit errors corrected by the RM(30,14) decoder */
	uint16_t raw;               /* the 14 information bits */
} tetra_rx_ev_aach_t;

/* MAC PDU kinds reported in tetra_rx_ev_signal_t.mac_pdu */
typedef enum {
	TETRA_RX_MAC_RESOURCE = 0,
	TETRA_RX_MAC_FRAG_END = 1, /* never reported on its own: reassembled */
	TETRA_RX_MAC_BROADCAST = 2, /* ACCESS-DEFINE and other non-SYSINFO broadcasts */
	TETRA_RX_MAC_D_BLCK = 3,
	TETRA_RX_MAC_U_SIGNAL = 4  /* user-to-user signalling on a stolen half slot,
	                              121 bit TM-SDU reported raw */
} tetra_rx_mac_pdu_t;

/* address types of MAC-RESOURCE (EN 300 392-2 table 21.55) */
typedef enum {
	TETRA_RX_ADDR_NULL = 0,
	TETRA_RX_ADDR_SSI = 1,
	TETRA_RX_ADDR_EVENT_LABEL = 2,
	TETRA_RX_ADDR_USSI = 3,
	TETRA_RX_ADDR_SMI = 4,
	TETRA_RX_ADDR_SSI_EVENT = 5,
	TETRA_RX_ADDR_SSI_USAGE = 6,
	TETRA_RX_ADDR_SMI_EVENT = 7
} tetra_rx_addr_type_t;

typedef struct {
	uint8_t type;          /* tetra_rx_addr_type_t */
	uint32_t ssi;          /* 24 bit; encrypted short identity if the PDU is encrypted */
	uint16_t event_label;  /* 10 bit */
	uint8_t usage_marker;  /* 6 bit */
} tetra_rx_addr_t;

typedef struct {
	uint8_t present;
	uint8_t alloc_type;    /* 0 replace, 1 additional, 2 quit and go, 3 replace + slot 1 */
	uint8_t timeslots;     /* bitmap, bit 3 = TN1 ... bit 0 = TN4 */
	uint8_t ul_dl;         /* 0 augmented, 1 DL, 2 UL, 3 both */
	uint8_t clch_permission;
	uint8_t cell_change;
	uint16_t carrier;      /* 12 bit carrier number */
	uint8_t ext_carrier;   /* 1: band/offset/duplex given in the PDU */
	uint32_t dl_freq_hz;   /* 0 if band unknown (no SYSINFO yet and no extension) */
} tetra_rx_chan_alloc_t;

/* party address used by CMCE PDUs */
typedef struct {
	uint8_t present;       /* 0 none, 1 SSI, 2 SSI + extension (TSI) */
	uint32_t ssi;
	uint16_t mcc;          /* extension */
	uint16_t mnc;          /* extension */
} tetra_rx_party_t;

/* MLE protocol discriminators (EN 300 392-2 18.5.21) */
#define TETRA_RX_PD_MM    1
#define TETRA_RX_PD_CMCE  2
#define TETRA_RX_PD_SNDCP 4
#define TETRA_RX_PD_MLE   5
#define TETRA_RX_PD_MGMT  6
#define TETRA_RX_PD_TEST  7

/* CMCE downlink PDU types (EN 300 392-2 table 14.60) */
#define TETRA_RX_CMCE_D_ALERT           0
#define TETRA_RX_CMCE_D_CALL_PROCEEDING 1
#define TETRA_RX_CMCE_D_CONNECT         2
#define TETRA_RX_CMCE_D_CONNECT_ACK     3
#define TETRA_RX_CMCE_D_DISCONNECT      4
#define TETRA_RX_CMCE_D_INFO            5
#define TETRA_RX_CMCE_D_RELEASE         6
#define TETRA_RX_CMCE_D_SETUP           7
#define TETRA_RX_CMCE_D_STATUS          8
#define TETRA_RX_CMCE_D_TX_CEASED       9
#define TETRA_RX_CMCE_D_TX_CONTINUE     10
#define TETRA_RX_CMCE_D_TX_GRANTED      11
#define TETRA_RX_CMCE_D_TX_WAIT         12
#define TETRA_RX_CMCE_D_TX_INTERRUPT    13
#define TETRA_RX_CMCE_D_CALL_RESTORE    14
#define TETRA_RX_CMCE_D_SDS_DATA        15
#define TETRA_RX_CMCE_D_FACILITY        16

/* tetra_rx_cmce_t.fields: which members were decoded from the PDU */
#define TETRA_RX_CMCE_F_CALL_ID       (1u << 0)
#define TETRA_RX_CMCE_F_BASIC_SERVICE (1u << 1)
#define TETRA_RX_CMCE_F_TX_GRANT      (1u << 2)
#define TETRA_RX_CMCE_F_TX_REQ_PERM   (1u << 3)
#define TETRA_RX_CMCE_F_PRIORITY      (1u << 4)
#define TETRA_RX_CMCE_F_PARTY         (1u << 5) /* calling / transmitting party */
#define TETRA_RX_CMCE_F_DISC_CAUSE    (1u << 6)
#define TETRA_RX_CMCE_F_STATUS        (1u << 7) /* pre-coded status */
#define TETRA_RX_CMCE_F_SDS           (1u << 8) /* sds_type and sds_data/sds_bits */
#define TETRA_RX_CMCE_F_TEXT          (1u << 9) /* text[] holds UTF-8 */
#define TETRA_RX_CMCE_F_ENC_CONTROL   (1u << 10)
#define TETRA_RX_CMCE_F_SIMPLEX       (1u << 11)
#define TETRA_RX_CMCE_F_HOOK          (1u << 12)
#define TETRA_RX_CMCE_F_CALL_TIMEOUT  (1u << 13)
#define TETRA_RX_CMCE_F_TEMP_ADDR     (1u << 14)
#define TETRA_RX_CMCE_F_NOTIFICATION  (1u << 15)
#define TETRA_RX_CMCE_F_CALL_OWNER    (1u << 16)
#define TETRA_RX_CMCE_F_PROTOCOL_ID   (1u << 17)

typedef struct {
	uint32_t fields;        /* TETRA_RX_CMCE_F_* */
	uint8_t parse_ok;       /* 0: only pdu_type is trustworthy (layout not parsed
	                           or inconsistent length); use the raw bits */
	uint16_t call_id;       /* 14 bit */
	uint8_t call_timeout;
	uint8_t hook_method;
	uint8_t duplex;         /* 0 simplex, 1 duplex */
	/* basic service information */
	uint8_t circuit_mode;   /* 0 = speech TCH/S */
	uint8_t e2e_encryption; /* end-to-end encryption flag of the basic service */
	uint8_t comm_type;      /* 0 p2p, 1 p2mp (group), 2 p2mp ack, 3 broadcast */
	uint8_t slots_or_speech; /* slots per frame, or speech service */
	uint8_t tx_grant;       /* 0 granted, 1 not granted, 2 queued, 3 granted to another */
	uint8_t tx_req_permission;
	uint8_t enc_control;    /* D-TX-GRANTED "encryption control" */
	uint8_t call_priority;
	uint8_t call_ownership;
	uint8_t notification;
	uint8_t disconnect_cause;
	uint32_t temp_address;  /* 24 bit */
	tetra_rx_party_t party; /* calling party (D-SETUP/D-STATUS/D-SDS-DATA) or
	                           transmitting party (D-TX-GRANTED/INTERRUPT) */
	uint16_t status;        /* pre-coded status */
	uint8_t sds_type;       /* short data type identifier 0..3 */
	uint8_t protocol_id;    /* SDS type 4 protocol identifier */
	uint8_t text_coding;    /* text coding scheme, when a text protocol was recognised */
	uint16_t sds_bits;      /* user data length in bits */
	const uint8_t *sds_data; /* user data, one bit per byte (0/1), sds_bits long */
	char text[TETRA_RX_TEXT_MAX]; /* NUL terminated UTF-8, only with F_TEXT */
} tetra_rx_cmce_t;

typedef struct {
	/* --- MAC --- */
	uint8_t mac_pdu;         /* tetra_rx_mac_pdu_t */
	uint8_t lchan;           /* tetra_rx_chan_t the PDU (first fragment) came in */
	tetra_rx_addr_t addr;
	uint8_t encryption_mode; /* 2 bit MAC-RESOURCE field, 0 = clear */
	uint8_t encrypted;       /* 1: TM-SDU is air-interface encrypted, not parsed */
	uint8_t fragmented;      /* 1: reassembled from MAC-RESOURCE + FRAG/END */
	uint8_t slot_grant;      /* 1: slot granting element present */
	tetra_rx_chan_alloc_t chan_alloc; /* not decodable (present=0) when encrypted */
	/* --- LLC --- */
	uint8_t have_llc;        /* 0: no TM-SDU (or encrypted) */
	uint8_t llc_pdu;         /* raw 4 bit LLC PDU type (table 21.1) */
	uint8_t llc_fcs_present;
	uint8_t llc_fcs_ok;
	/* --- MLE --- */
	uint8_t have_mle;        /* 0: LLC PDU carried no (complete) TL-SDU */
	uint8_t pdisc;           /* TETRA_RX_PD_* */
	uint8_t pdu_type;        /* PDU type within that protocol (CMCE 5 bit, MM 4 bit, MLE 3 bit, SNDCP 4 bit) */
	const char *pdu_name;    /* static string, never NULL */
	/* --- CMCE --- */
	uint8_t have_cmce;       /* cmce is valid */
	tetra_rx_cmce_t cmce;
	/* --- raw TL-SDU (starting at the protocol discriminator) or, if there is
	 * no LLC SDU, the TM-SDU; one bit per byte. Valid during the callback. */
	const uint8_t *sdu;
	uint16_t sdu_bits;
} tetra_rx_ev_signal_t;

typedef enum {
	TETRA_RX_ENC_UNKNOWN = 0, /* no signalling seen for this channel yet */
	TETRA_RX_ENC_CLEAR = 1,
	TETRA_RX_ENC_ENCRYPTED = 2
} tetra_rx_enc_state_t;

typedef struct {
	uint8_t usage_marker;   /* traffic usage marker from the AACH (4..63) */
	uint8_t n_frames;       /* number of speech frames delivered: 0, 1 or 2 */
	uint8_t stolen[2];      /* half slot i was stolen for signalling (STCH) */
	uint8_t bfi[2];         /* bad frame indicator of frame i (CRC failure or stolen) */
	uint8_t enc_state;      /* tetra_rx_enc_state_t, from signalling only */
	uint8_t crc_ok;         /* class 2 CRC of the slot */
	/* frame[i]: 137 codec bits in EN 300 395-2 codec order, one bit per byte.
	 * If enc_state == TETRA_RX_ENC_ENCRYPTED the bits are still encrypted. */
	uint8_t frame[2][TETRA_RX_SPEECH_FRAME_BITS];
	/* Descrambled type-4 soft bits of the traffic part of the slot (before
	 * de-interleaving): 432 for a full slot, 216 (second half slot) if the
	 * first half was stolen, 0 if both were. int8, positive = 0, negative = 1,
	 * magnitude = reliability. This is what the ETSI reference channel
	 * decoder ("cdecoder") takes as input, see README. Valid during the
	 * callback. */
	const int8_t *soft;
	uint16_t n_soft;
} tetra_rx_ev_traffic_t;

typedef struct {
	uint8_t chan;           /* tetra_rx_chan_t */
	uint8_t blk_num;        /* 0 full slot / AACH, 1 or 2 half slot */
	uint8_t crc_ok;
	uint16_t n_bits;        /* type-1 bits */
	const uint8_t *bits;    /* one bit per byte */
} tetra_rx_ev_block_t;

/* One demodulated slot before channel decoding (diagnostics, feeding other
 * decoders). */
typedef struct {
	uint8_t burst_type;     /* 0 normal burst, training sequence 1 (one logical
	                           channel); 1 normal burst, training sequence 2 (two
	                           logical channels / stolen); 2 synchronisation burst */
	float sync_score;       /* 0..1 correlation with the training sequences */
	uint16_t n_bits;        /* 510 */
	const int8_t *soft;     /* positive = 0, negative = 1 */
} tetra_rx_ev_burst_t;

typedef struct {
	tetra_rx_event_type_t type;
	tetra_rx_time_t time;   /* slot the event belongs to */
	union {
		tetra_rx_ev_sync_t sync;
		tetra_rx_ev_bsch_t bsch;
		tetra_rx_ev_sysinfo_t sysinfo;
		tetra_rx_ev_aach_t aach;
		tetra_rx_ev_signal_t signal;
		tetra_rx_ev_traffic_t traffic;
		tetra_rx_ev_block_t block;
		tetra_rx_ev_burst_t burst;
	} u;
} tetra_rx_event_t;

/* Called synchronously from tetra_rx_process_*(). Pointers inside the event
 * are only valid during the call. Do not call tetra_rx_* from the callback
 * except tetra_rx_get_status(). */
typedef void (*tetra_rx_event_cb_t)(void *user, const tetra_rx_event_t *ev);

/* ---- configuration ---------------------------------------------------- */
#define TETRA_RX_FLAG_AACH_EVENTS  (1u << 0) /* emit TETRA_RX_EV_AACH for every slot */
#define TETRA_RX_FLAG_BLOCK_EVENTS (1u << 1) /* emit TETRA_RX_EV_BLOCK for every block */
#define TETRA_RX_FLAG_DC_BLOCK     (1u << 2) /* remove a DC component before demodulation */
#define TETRA_RX_FLAG_NO_TRAFFIC   (1u << 3) /* do not channel-decode TCH/S */
#define TETRA_RX_FLAG_BURST_EVENTS (1u << 4) /* emit TETRA_RX_EV_BURST for every slot */

typedef struct {
	tetra_rx_event_cb_t event_cb; /* may be NULL (status polling only) */
	void *user;
	uint32_t flags;               /* TETRA_RX_FLAG_* */
	/* optional allocator used once in create/destroy (e.g. PSRAM);
	 * NULL selects malloc/free */
	void *(*alloc)(size_t size);
	void (*free)(void *ptr);
} tetra_rx_config_t;

/* ---- status ----------------------------------------------------------- */
typedef struct {
	/* 1. lock state and signal metrics */
	uint8_t locked;             /* burst synchronisation (training sequences found
	                               at the slot rate). Decoding is confirmed by
	                               time.valid and the chan[] counters: a signal
	                               that is not a TMO downlink can lock briefly
	                               but never sets time.valid. */
	float freq_offset_hz;       /* carrier offset estimate: the carrier is this far
	                               from 0 Hz in the input (retune by this amount to
	                               centre it). Last value is kept after a loss. */
	float timing_ppm;           /* sample clock offset estimate while locked;
	                               positive = more than 4 input samples per symbol,
	                               i.e. the input sample clock is fast relative to
	                               the base station */
	float level_dbfs;           /* in-channel power after the matched filter,
	                               dB relative to full scale 1.0 */
	float evm_snr_db;           /* SNR estimate from the differential constellation */
	float train_ber;            /* smoothed training sequence bit error rate */
	uint32_t train_bits;        /* training bits compared since create/reset */
	uint32_t train_bit_errors;
	uint32_t slots;             /* slots processed while locked */
	uint32_t lock_count;        /* number of lock acquisitions */
	uint32_t sync_losses;
	tetra_rx_chan_stat_t chan[TETRA_RX_CH_COUNT];
	uint32_t frag_dropped;      /* MAC fragments discarded (lost start/end, overflow) */
	/* 2. cell identity */
	uint8_t cell_valid;         /* a SYNC PDU (BSCH, CRC ok) was decoded since
	                               create/reset; mcc/mnc/colour_code are those of
	                               the last one and are kept across sync losses.
	                               time.valid is the "decoding now" indication:
	                               it is cleared on every sync loss. */
	uint16_t mcc, mnc;
	uint8_t colour_code;
	tetra_rx_time_t time;       /* time of the last processed slot */
	/* 3. broadcast */
	uint8_t sysinfo_valid;
	tetra_rx_ev_sysinfo_t sysinfo;
	/* per timeslot (index 0 = TN1) usage from the last decoded ACCESS-ASSIGN
	 * of frames 1..17. If an ACCESS-ASSIGN cannot be decoded, a timeslot that
	 * carried traffic is treated as traffic for up to 2 more frames. */
	struct {
		uint8_t valid;
		uint8_t dl_usage;       /* TETRA_RX_DL_* or traffic usage marker */
		uint8_t ul_usage;
		uint8_t enc_state;      /* tetra_rx_enc_state_t for traffic on this slot:
		                           what the signalling of the call (usage marker,
		                           stolen half slots) showed last; kept for the
		                           duration of the call, UNKNOWN again when the
		                           traffic usage leaves the timeslot */
	} slot[4];
	uint64_t samples;           /* complex samples consumed */
} tetra_rx_status_t;

/* ---- API -------------------------------------------------------------- */

/* Allocate a receiver. All memory is one allocation made here (about 35 kB:
 * 35248 bytes on RV32, see README); tetra_rx_process_* never allocates and
 * needs about 1.3 kB of stack plus what the event callback uses. Returns
 * NULL on failure. */
tetra_rx_t *tetra_rx_create(const tetra_rx_config_t *cfg);
void tetra_rx_destroy(tetra_rx_t *rx);

/* Drop synchronisation and all protocol state (e.g. after retuning). */
void tetra_rx_reset(tetra_rx_t *rx);

/* Feed n_complex samples of interleaved I,Q at 72 kHz. Any scale is accepted
 * (the receiver normalises); level_dbfs assumes full scale = 1.0. */
void tetra_rx_process_cf32(tetra_rx_t *rx, const float *iq, size_t n_complex);
/* Same for interleaved int16 I,Q (full scale 32768). */
void tetra_rx_process_cs16(tetra_rx_t *rx, const int16_t *iq, size_t n_complex);

/* Copy the current status. May be called from the event callback or between
 * process calls from the same task (the library does no locking: one receiver
 * must only be used from one task at a time). */
void tetra_rx_get_status(const tetra_rx_t *rx, tetra_rx_status_t *st);

/* Copy the most recent differential symbols, the values the detector decides
 * on (matched filter, symbol timing, s[k] * conj(s[k-1])), as interleaved
 * re, im, oldest first, scaled to an RMS magnitude of 1. A clean signal gives
 * four points at (+-0.71, +-0.71); a carrier offset that the receiver has not
 * removed (yet) turns the pattern, noise blurs it. At most
 * TETRA_RX_CONSTELLATION_MAX symbols; returns the number copied. Same task
 * rule as tetra_rx_get_status(). */
#define TETRA_RX_CONSTELLATION_MAX 512
size_t tetra_rx_get_constellation(const tetra_rx_t *rx, float *iq, size_t max_symbols);

/* Static names for logging. */
const char *tetra_rx_chan_name(uint8_t chan);
const char *tetra_rx_pdisc_name(uint8_t pdisc);
const char *tetra_rx_version(void);

#ifdef __cplusplus
}
#endif
#endif /* TETRA_RX_H */
