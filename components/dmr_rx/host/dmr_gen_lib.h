/*
 * dmr_rx host tools - synthetic DMR signal generator.
 *
 * Builds valid DMR bursts with the encoder classes of MMDVMHost (slot type,
 * BPTC(196,96), RS(12,9) full LC, embedded LC, EMB, short LC, CRC-CCITT), a
 * CACH/TACT builder that follows the MMDVM modem firmware, and real AMBE+2
 * frames received off air; then 4FSK-modulates them (root raised cosine,
 * alpha 0.2, 4800 symbols/s) into what an FM discriminator delivers at
 * 48 kS/s, with optional noise, DC offset, gain, inversion and sample clock
 * error.
 *
 * Host only (uses the C++ standard library).
 *
 * SPDX-License-Identifier: AGPL-3.0-only
 */
#ifndef DMR_GEN_LIB_H
#define DMR_GEN_LIB_H

#include <stdint.h>
#include <string>
#include <vector>

namespace dmrgen {

enum Mode { MODE_BS = 0, MODE_MS = 1, MODE_DM1 = 2, MODE_DM2 = 3 };

struct BurstSpec {
    bool tx = false;       /* false: nothing transmitted in this slot       */
    bool busy = false;     /* voice / data traffic (sets the TACT AT bit)   */
    uint8_t bits[33] = {}; /* 264 burst bits, MSB first                     */
    int data_type = -1;    /* data bursts: the slot type's data type        */
    int voice_index = -1;  /* voice bursts: ordinal within the script (its
                              frames are sent_ambe[27 * voice_index ...])   */
};

struct CallSpec {
    uint32_t src = 1234567;
    uint32_t dst = 91;
    bool group = true;
    bool encrypted = false;   /* privacy bit in the LC + PI header + EMB PI  */
    bool emergency = false;
    uint8_t fid = 0;
    int headers = 2;          /* voice LC headers (0: late entry)            */
    int superframes = 4;      /* 6 voice bursts each                         */
    bool terminator = true;
    std::string alias;        /* talker alias sent in the embedded LC        */
    unsigned first_frame = 0; /* index into the AMBE test vectors            */
};

/* A data packet: header, optional proprietary header, data blocks. */
struct PacketSpec {
    uint32_t src = 2308094;
    uint32_t dst = 2308092;
    bool group = false;
    bool confirmed = false;   /* blocks with serial number and CRC-9         */
    int response = -1;        /* the header's A bit; -1: as confirmed        */
    uint8_t data_type = 7;    /* 7 rate 1/2, 8 rate 3/4, 10 rate 1           */
    uint8_t dpf = 0;          /* 0: 3 if confirmed, else 2; 13 defined short
                                 data, 14 raw short data                     */
    uint8_t sap = 4;
    uint8_t dd_format = 0;    /* dpf 13                                      */
    int prop_mfid = -1;       /* >= 0: a proprietary header with this MFID
                                 follows the header ...                      */
    uint8_t prop_sap = 9;     /* ... its SAP ...                             */
    uint8_t prop_data[8] = {}; /* ... and its other eight octets             */
    std::vector<uint8_t> data;
};

/* One timeslot's sequence of bursts (one entry per 60 ms). */
class SlotScript {
public:
    SlotScript(Mode mode, uint8_t cc) : m_mode(mode), m_cc(cc) {}

    void silence(int n);                 /* no transmission                  */
    void idle(int n);
    void csbk(const uint8_t pdu10[10], int n = 1);
    void data_header(const uint8_t pdu10[10]);
    void pi_header(const uint8_t pdu10[10]);
    void call(const CallSpec &c);
    /* raw: any data type with a ready made 12 byte BPTC payload */
    void data_raw(uint8_t data_type, const uint8_t pdu12[12], bool busy = true);
    /* one data block: 12 (rate 1/2), 18 (rate 3/4) or 24 octets (rate 1) */
    void data_block(uint8_t data_type, const uint8_t *blk);
    /* a burst as it was received somewhere (33 octets) */
    void burst_raw(const uint8_t bits33[33], int data_type = -1);
    /* a whole packet; returns the number of bursts it took */
    int packet(const PacketSpec &p);
    /* a status / precoded message: a header and nothing else */
    void status(uint32_t src, uint32_t dst, bool group, unsigned status10);

    std::vector<BurstSpec> bursts;
    /* every AMBE frame (9 bytes) put into voice bursts, in order */
    std::vector<uint8_t> sent_ambe;

private:
    void finish_data(BurstSpec &b, uint8_t data_type);
    Mode m_mode;
    uint8_t m_cc;
};

struct Channel {
    bool noise = false;
    double snr_db = 30.0;   /* signal power / noise power in the full 24 kHz
                               bandwidth of the 48 kS/s discriminator signal
                               (Es/N0 after the matched filter is ~10 dB
                               more)                                         */
    double gain = 1.0;      /* 1.0: samples in rad/sample (+-1944 Hz ->
                               +-0.2545)                                     */
    double dc = 0.0;        /* added after the gain                          */
    bool invert = false;
    double ppm = 0.0;       /* receiver sample clock error (+: clock fast)   */
    uint32_t seed = 1;
    int lead_symbols = 480; /* nothing transmitted before / after            */
    int tail_symbols = 480;
    bool noise_in_gaps = true; /* un-squelched discriminator: with no carrier
                                  the noise is ~10 dB above the outer
                                  deviation                                  */
};

/* slot1 / slot2: scripts of the two timeslots (either may be empty). In
 * MODE_BS the CACH is transmitted; short_lc (4 bytes: SLCO in the low nibble
 * of byte 0, 24 data bits) is spread over it when not NULL. */
std::vector<float> modulate(Mode mode, const std::vector<BurstSpec> &slot1, const std::vector<BurstSpec> &slot2,
                            const uint8_t *short_lc, const Channel &ch);

/* CACH for one burst (MMDVM firmware layout). frag17: 17 payload bits. */
void build_cach(bool at, bool tc, unsigned lcss, const uint8_t frag17[17], uint8_t cach[3]);

/* Root raised cosine impulse response, t in symbols, unit energy. */
double rrc(double t, double alpha);

} // namespace dmrgen

#endif
