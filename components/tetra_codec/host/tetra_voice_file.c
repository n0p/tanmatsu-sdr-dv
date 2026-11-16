// SPDX-License-Identifier: AGPL-3.0-only
// tetra_voice_file - decode the speech of a TETRA downlink recording to a WAV file.
//
//   tetra_voice_file [--tn N] IN.cs16 OUT.wav        (IN: 72 kHz int16 I/Q, as "tsdr.py cap ch")
//
// The recording goes through tetra_rx; the speech frames of one timeslot go through the
// tetra_codec wrapper, the same path as in the firmware. In addition the soft bits of every
// traffic slot are channel-decoded a second time by the ETSI reference channel decoder, and its
// output is compared with the frames tetra_rx delivered: this checks the bit order tetra_rx hands
// to the codec against the reference.
// (ref_adapt.h is force-included: Word16 / Word32 and the standard headers.)
#include "channel.h"
#include "tetra_codec.h"
#include "tetra_rx.h"

#undef printf
#undef exit

#define L TETRA_CODEC_FRAME_SAMPLES

static struct {
    tetra_rx_t* rx;
    FILE*       out;
    int         tn;          // 1..4, 0 = the first timeslot that carries speech
    uint64_t    last_sample; // input position of the last speech slot
    uint64_t    written;     // PCM samples written
    int         first_pass;
    unsigned    slots, slots_enc, frames, frames_bad, spurts;
    unsigned    stolen_none, stolen_first, stolen_second, stolen_both;
    unsigned    ref_slots, ref_frames, ref_frame_match, ref_bfi_match, ref_bit_diff;
    double      energy;
} g;

static void write_pcm(const int16_t* pcm, int n) {
    fwrite(pcm, sizeof(int16_t), (size_t)n, g.out);
    g.written += (uint64_t)n;
}

// Second opinion on the channel decoding: the reference decoder on the same soft bits.
static void reference_check(const tetra_rx_ev_traffic_t* t) {
    Word16 in[432], coded[432], frames[286];
    if (t->n_soft != 432 || t->stolen[0] || t->stolen[1]) return;
    for (int i = 0; i < 432; i++) in[i] = t->soft[i] < -127 ? -127 : t->soft[i];
    Desinterleaving_Speech(in, coded);
    const Word16 bfi = Channel_Decoding((short)g.first_pass, 0, coded, frames);
    g.first_pass     = 0;
    g.ref_slots++;
    if ((bfi != 0) == (t->bfi[0] != 0)) g.ref_bfi_match++;
    if (bfi || t->bfi[0] || t->bfi[1] || t->n_frames != 2) return;
    for (int f = 0; f < 2; f++) {
        int diff = 0;
        for (int i = 0; i < 137; i++) diff += (frames[137 * f + i] != 0) != (t->frame[f][i] != 0);
        g.ref_frames++;
        g.ref_bit_diff += (unsigned)diff;
        if (diff == 0) g.ref_frame_match++;
    }
}

static void on_event(void* user, const tetra_rx_event_t* ev) {
    (void)user;
    if (ev->type != TETRA_RX_EV_TRAFFIC) return;
    const tetra_rx_ev_traffic_t* t = &ev->u.traffic;
    if (g.tn == 0) g.tn = ev->time.tn;
    if (ev->time.tn != g.tn) return;
    if (t->enc_state == TETRA_RX_ENC_ENCRYPTED) {
        g.slots_enc++;
        return;
    }
    reference_check(t);

    tetra_rx_status_t st;
    tetra_rx_get_status(g.rx, &st);
    // Keep the WAV on the time axis of the recording: a pause in the traffic becomes silence,
    // and the decoder starts from its initial state after it.
    const uint64_t pos = st.samples * 8000 / TETRA_RX_SAMPLE_RATE_HZ;
    if (g.slots == 0 || st.samples - g.last_sample > TETRA_RX_SAMPLE_RATE_HZ / 2) {
        static const int16_t zero[L];
        while (g.written + L <= pos) write_pcm(zero, L);
        tetra_codec_reset();
        g.spurts++;
    }
    g.last_sample = st.samples;
    g.slots++;
    if (t->stolen[0] && t->stolen[1]) g.stolen_both++;
    else if (t->stolen[0]) g.stolen_first++;
    else if (t->stolen[1]) g.stolen_second++;
    else g.stolen_none++;

    for (int f = 0; f < 2; f++) {
        int16_t    pcm[L];
        const bool bad = f >= t->n_frames || t->bfi[f] || t->stolen[f];
        tetra_codec_decode(t->frame[f], bad, pcm);
        write_pcm(pcm, L);
        g.frames++;
        g.frames_bad += bad;
        for (int i = 0; i < L; i++) g.energy += (double)pcm[i] * pcm[i];
    }
}

static void put32(uint8_t* p, uint32_t v) {
    for (int i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (8 * i));
}

int main(int argc, char** argv) {
    const char *in_path = NULL, *out_path = NULL;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--tn") && i + 1 < argc) g.tn = atoi(argv[++i]);
        else if (!in_path) in_path = argv[i];
        else if (!out_path) out_path = argv[i];
    }
    if (!in_path || !out_path || !tetra_codec_available()) {
        fprintf(stderr, "usage: tetra_voice_file [--tn N] IN.cs16 OUT.wav   (72 kHz int16 I/Q)\n");
        return 2;
    }
    FILE* in = fopen(in_path, "rb");
    g.out    = fopen(out_path, "wb");
    if (!in || !g.out) {
        perror(in ? out_path : in_path);
        return 1;
    }
    uint8_t hdr[44] = "RIFF....WAVEfmt ";
    put32(hdr + 16, 16);
    hdr[20] = 1, hdr[22] = 1;  // PCM, mono
    put32(hdr + 24, 8000);
    put32(hdr + 28, 16000);
    hdr[32] = 2, hdr[34] = 16;
    memcpy(hdr + 36, "data", 4);
    fwrite(hdr, 1, 44, g.out);

    tetra_rx_config_t cfg = {.event_cb = on_event};
    g.rx                  = tetra_rx_create(&cfg);
    g.first_pass          = 1;
    static int16_t buf[2 * 1024];
    size_t         n;
    while ((n = fread(buf, 2 * sizeof(int16_t), 1024, in)) > 0) tetra_rx_process_cs16(g.rx, buf, n);
    tetra_rx_status_t st;
    tetra_rx_get_status(g.rx, &st);

    put32(hdr + 4, (uint32_t)(36 + g.written * 2));
    put32(hdr + 40, (uint32_t)(g.written * 2));
    fseek(g.out, 0, SEEK_SET);
    fwrite(hdr, 1, 44, g.out);
    fclose(g.out);

    printf("input %.1f s, timeslot %d: %u speech slots in %u talk spurt(s), %u encrypted slots skipped\n",
           (double)st.samples / TETRA_RX_SAMPLE_RATE_HZ, g.tn, g.slots, g.spurts, g.slots_enc);
    printf("half slots stolen: none %u, first %u, second %u, both %u\n", g.stolen_none, g.stolen_first,
           g.stolen_second, g.stolen_both);
    printf("speech frames %u, of which %u bad or stolen (concealed by the decoder)\n", g.frames, g.frames_bad);
    printf("reference channel decoder on the same soft bits: %u slots, bad frame flag agrees on %u;\n"
           "  %u frames compared, %u identical, %u bits differ\n",
           g.ref_slots, g.ref_bfi_match, g.ref_frames, g.ref_frame_match, g.ref_bit_diff);
    printf("output %.1f s of 8 kHz audio, rms %.0f -> %s\n", (double)g.written / 8000.0,
           g.frames ? sqrt(g.energy / ((double)g.frames * L)) : 0.0, out_path);
    return 0;
}
