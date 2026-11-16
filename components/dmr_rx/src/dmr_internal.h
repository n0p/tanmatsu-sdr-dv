/*
 * dmr_rx - internal definitions.
 *
 * SPDX-License-Identifier: AGPL-3.0-only
 */
#ifndef DMR_RX_INTERNAL_H
#define DMR_RX_INTERNAL_H

#include <atomic>
#include <stddef.h>
#include <stdint.h>

#include "dmr_rx.h"

#include "BPTC19696.h"
#include "DMREmbeddedData.h"
#include "DMRShortLC.h"
#include "dmr_fec.h"

namespace dmrrx {

/* ------------------------------------------------------------------------ */
/* Demodulator: 48 kS/s discriminator -> RRC, decimate by 2 -> 24 kS/s ring */
/* ------------------------------------------------------------------------ */

enum {
    kSps        = 5,    /* samples per symbol at 24 kS/s                      */
    kRingBits   = 11,
    kRingSize   = 1 << kRingBits,
    kRingMask   = kRingSize - 1,
    kFirTaps    = 81,   /* RRC, +-4 symbols at 48 kS/s                        */
    kFirHist    = 80 + 256,
    kSyncSyms   = 24,
    kBurstSyms  = 132,
    kCachSyms   = 12,
    kTotalSyms  = kBurstSyms + kCachSyms,
    kSyncEndSym = 77,   /* burst symbol index of the last sync symbol         */
    kPeriod     = 1440, /* one TDMA frame (60 ms) in 24 kS/s samples          */
    kNumFamilies = 4
};

/* Time: Q16 fixed point, unit = one 24 kS/s sample. */
typedef int64_t tq_t;
enum : int64_t { kOne = 65536 };

enum Family : uint8_t { kFamBS = 0, kFamMS = 1, kFamDM1 = 2, kFamDM2 = 3 };

/* Voice sync patterns (48 bit) of the four families; data = bitwise inverse
 * of every dibit's MSB, i.e. symbol negation. */
extern const uint8_t kVoiceSync[kNumFamilies][6];

struct SyncDet {
    tq_t t_end;   /* time of the last sync symbol                             */
    uint8_t family;
    int8_t sign;  /* +1: matches the voice pattern, -1: the data pattern      */
    float rho2;   /* squared normalised correlation, 0..1                     */
    float g3;     /* outer symbol level                                       */
    float d;      /* DC offset                                                */
};

struct Burst {
    uint8_t bits[kBurstBytes]; /* 264 bits, normal polarity, MSB first        */
    uint8_t cach[3];           /* 24 CACH bits preceding the burst            */
    float g, d;                /* outer level, DC offset used for slicing     */
    float evm;                 /* RMS slicer error / outer level              */
    float tau;                 /* timing correction applied (samples)         */
};

struct Demod {
    float taps[kFirTaps / 2 + 1];
    float hist[kFirHist];
    int hist_n;
    int phase;
    float dc_slow;
    bool dc_init;

    float ring[kRingSize];
    int64_t n; /* 24 kS/s samples written so far                              */

    float sgn[kNumFamilies][kSyncSyms];

    /* sync peak search */
    bool pk_active;
    int64_t pk_n;
    uint8_t pk_fam;
    float pk_rho2, pk_c0, pk_cm, pk_cp, pk_d;
    float c_prev[kNumFamilies];
    int64_t holdoff_until;

    void init();
    void reset();
    /* Push one 48 kS/s sample. Returns true when a new 24 kS/s sample was
     * produced (it is then the newest ring entry). */
    bool push(float x);
    /* Run the sync correlators on the newest sample. */
    bool search(SyncDet &det);
    /* Slice the burst whose last sync symbol is at t_end. If family >= 0 the
     * levels are taken from the sync word (pattern sign given by sign),
     * otherwise g/d are the initial levels. invert flips the polarity. */
    void slice(tq_t t_end, int family, int sign, float g, float d, Burst &out) const;

    float sample_cost(tq_t t0, float g, float d, float *x) const;
    void sample(tq_t t0, float *x) const;
};

/* ------------------------------------------------------------------------ */
/* Receiver state                                                           */
/* ------------------------------------------------------------------------ */

struct TalkerAlias {
    uint8_t buf[28];
    uint8_t have; /* bit per 7 byte block */
    char text[64];
    uint8_t complete;
};

struct Call {
    bool active;
    bool ids_valid;
    bool late_entry;
    bool group;
    bool encrypted;
    bool emergency;
    uint8_t flco, fid, svc;
    uint32_t dst, src;
    uint32_t voice_bursts;
    /* candidate for an id change seen in the embedded LC */
    uint32_t cand_dst, cand_src;
    uint8_t cand_n;
    uint8_t data_in_call;
};

struct Vocoder;

struct Slot {
    bool active;
    bool confirmed;
    uint8_t family;
    uint8_t label;      /* 0 / 1 */
    bool label_known;
    int8_t tc_votes;
    uint8_t tc_contra; /* error free TACTs in a row against the label */
    bool tc_clean;     /* latest TACT: no errors, and it agrees with the label */
    tq_t t_next;        /* predicted time of the last sync symbol            */
    tq_t t_last_sync;
    uint16_t periods_since_sync;
    bool hit;
    SyncDet det;
    bool has_pend;      /* a strong sync was seen off the predicted timing    */
    SyncDet pend;
    uint8_t state;      /* dmr_slot_state_t                                   */
    uint8_t vindex;     /* voice superframe burst index 0..5                  */
    uint8_t misses;
    uint8_t unconf_age;
    uint8_t unconf_syncs;
    bool prov_voice;
    float g, d;
    uint8_t cc;
    uint8_t last_data_type;
    uint8_t sync_type;
    uint8_t last_sync_errs;
    bool pending_enc;   /* PI header seen before the voice                    */

    Call call;
    uint8_t last_emb[9];
    bool have_last_emb;
    TalkerAlias ta;

    float ber, evm;
    bool q_init;
};

struct Totals {
    uint32_t voice_bursts, data_bursts, missed_bursts, fec_errs, fec_bits;
};

struct ShortLcAsm {
    uint8_t bits[68];
    uint8_t n;
    tq_t t_last;
    uint8_t last[5];
    bool have_last;
};

/* A data packet being put together: the header and the blocks so far. */
enum {
    kDataBlockMax  = 24, /* bytes in a rate 1 block                           */
    kDataMaxBlocks = 64, /* blocks kept; what follows of a longer packet is
                            dropped                                           */
    kPduTextMax    = 768
};
struct DataAsm {
    bool active;
    bool prop_lost;      /* a proprietary header was not readable             */
    uint8_t hdr[10];
    uint8_t prop[10];    /* first proprietary header                          */
    uint8_t props;       /* proprietary headers seen                          */
    uint8_t type;        /* data type of the blocks, 0xFF not known yet       */
    uint8_t announced;   /* blocks to follow according to the header          */
    uint8_t left;        /* ... of which are still to come                    */
    uint8_t n;           /* data blocks passed (received or not)              */
    uint8_t missing;     /* ... of which were not received                    */
    uint64_t have;       /* bit i: blk[i] was received                        */
    uint8_t blk[kDataMaxBlocks][kDataBlockMax]; /* must be the last member    */
};

struct Rx {
    dmr_rx_config_t cfg;
    Demod dem;
    Slot slot[2];
    CDMREmbeddedData emb[2]; /* embedded LC assembler per slot[] entry       */
    DataAsm dasm[2];         /* data packet assembler per slot[] entry       */
    uint8_t pdu_buf[kDataMaxBlocks * kDataBlockMax]; /* packet handed out    */
    char pdu_text[kPduTextMax];                      /* ... and its text     */
    Totals tot[2];           /* per timeslot label                           */
    tq_t period;        /* TDMA frame length estimate                         */

    /* polarity */
    bool inverted;
    bool pol_locked;
    uint8_t pol_votes;
    uint8_t pol_contra;

    ShortLcAsm slc;
    CBPTC19696 bptc;
    CDMRShortLC shortlc;

    uint64_t samples_in;
    uint32_t sync_count, sync_loss_count;
    float last_g, last_d;

    /* quality of the burst being processed (copied into events) */
    dmr_burst_quality_t q;
    uint64_t cur_sample_index;

    Vocoder *voc[2];

    /* status snapshot: two buffers, generation counter */
    dmr_rx_status_t status[2];
    std::atomic<uint32_t> status_gen;

    void init(const dmr_rx_config_t *cfg);
    void reset();
    void input(float x);
    void publish_status();

  private:
    void on_sync(const SyncDet &det);
    void process_burst(Slot &s);
    void slot_init(Slot &s, const SyncDet &det);
    void slot_drop(Slot &s, bool lost);
    void confirm(Slot &s);
    void assign_label(Slot &s);
    void relabel_other(Slot &o);
    void short_lc_add(Slot &s, uint8_t lcss, const uint8_t *payload17, tq_t t);
    void handle_data(Slot &s, const uint8_t *w, const struct DataEval &de, uint8_t sync_errs);
    void data_header(Slot &s, const uint8_t *hdr);
    void data_header_bad(Slot &s);
    void data_block(Slot &s, const uint8_t *w, uint8_t data_type, const struct DataEval &de);
    void data_missing(Slot &s);
    void data_flush(Slot &s);
    void data_finish(Slot &s);
    bool handle_voice(Slot &s, const uint8_t *w, const struct VoiceEval &ve, uint8_t index, bool has_sync,
                      uint8_t sync_errs);
    void handle_lc(Slot &s, const uint8_t raw[9], uint8_t source);
    void handle_talker_alias(Slot &s, uint8_t block, const uint8_t *data7);
    void call_start(Slot &s, bool late);
    void call_end(Slot &s);
    void call_update(Slot &s);
    void emit(Slot &s, dmr_event_t &ev, uint8_t type);
    void quality_update(Slot &s, bool miss);
};

/* Strength of a burst under the hypothesis "data" / "voice". */
struct DataEval {
    SlotType st;
    Bptc bp;
    bool have_bptc;
    bool crc_ok;     /* CRC / RS of the PDU verified                          */
    bool strong;
};
struct VoiceEval {
    AmbeFrame fr[3];
    uint8_t frame[3][9];
    unsigned errs;   /* sum of the three frames                               */
    bool strong;
};

void eval_data(CBPTC19696 &bptc, const uint8_t *w, DataEval &e);
void eval_voice(const uint8_t *w, VoiceEval &e);
unsigned sync_errors(const uint8_t *w, uint8_t family, bool data);

bool talker_alias_decode(const TalkerAlias &ta, char out[64], uint8_t &complete);

/* ------------------------------------------------------------------------ */
/* Vocoder (dmr_voice.cpp); only with DMR_RX_WITH_VOICE                      */
/* ------------------------------------------------------------------------ */
size_t vocoder_size();
void vocoder_init(Vocoder *v);
void vocoder_decode(Vocoder *v, const char ambe_d[49], int errs, bool bad, float gain,
                    int16_t pcm[DMR_RX_PCM_FRAME_SAMPLES]);

} // namespace dmrrx

struct dmr_rx {
    dmrrx::Rx rx;
};

#endif
