/* tetra_rx_file - decode a TETRA downlink IQ recording and print the events
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "tetra_rx.h"
#include "dsp_util.h"
#include "print_events.h"

enum fmt { F_CF32, F_CS16, F_CU8, F_CS8, F_BITS };

static struct {
	int quiet;
	FILE *dump_soft;
	FILE *dump_bits;
	uint64_t n_events[16];
	uint64_t hash;
	unsigned int tch_enc[3];
} g;

static void on_event(void *user, const tetra_rx_event_t *ev)
{
	char line[8192];

	(void) user;
	if (ev->type < 16)
		g.n_events[ev->type]++;
	if (ev->type == TETRA_RX_EV_BURST) {
		if (g.dump_bits) {
			uint8_t hard[510];
			unsigned int i;

			for (i = 0; i < 510; i++)
				hard[i] = ev->u.burst.soft[i] < 0;
			fwrite(hard, 1, 510, g.dump_bits);
		}
		return;
	}
	if (ev->type == TETRA_RX_EV_TRAFFIC) {
		const tetra_rx_ev_traffic_t *t = &ev->u.traffic;

		g.tch_enc[t->enc_state % 3]++;
		if (g.dump_soft && t->n_soft) {
			uint8_t hdr[8] = { (uint8_t) (t->n_soft / 216), t->usage_marker, ev->time.tn,
					   ev->time.fn, ev->time.mn, t->enc_state, 0, 0 };
			int8_t pad[432] = { 0 };

			memcpy(pad, t->soft, t->n_soft);
			fwrite(hdr, 1, 8, g.dump_soft);
			fwrite(pad, 1, 432, g.dump_soft);
		}
	}
	format_event(line, sizeof(line), ev);
	g.hash = fnv1a(g.hash, line);
	if (!g.quiet)
		puts(line);
}

static void usage(void)
{
	fprintf(stderr,
		"usage: tetra_rx_file [options] FILE\n"
		"  --format cf32|cs16|cu8|cs8|wav|bits  sample format (default: wav if RIFF header, else cf32)\n"
		"                    bits = demodulated bits, one per byte (re-modulated internally)\n"
		"  --rate HZ         sample rate of the file (default 72000, WAV: from header);\n"
		"                    anything else is resampled to 72000 here in the CLI\n"
		"  --swap-iq         exchange I and Q (inverted spectrum)\n"
		"  --shift HZ        shift the input by HZ before decoding\n"
		"  --skip N          skip N complex samples at the start\n"
		"  --max N           stop after N complex input samples\n"
		"  --block N         samples per tetra_rx_process call (default 1024)\n"
		"  --cs16-api        feed through tetra_rx_process_cs16 (needs 72000 Hz cs16 input)\n"
		"  --aach            print every ACCESS-ASSIGN\n"
		"  --blocks          print every decoded block\n"
		"  --no-traffic      do not decode TCH/S\n"
		"  --dc-block        enable the DC blocker\n"
		"  --dump-bits FILE  write the demodulated slots as hard bits (1 byte/bit), e.g. for osmo-tetra's tetra-rx\n"
		"  --dump-soft FILE  write traffic slot soft bits (8 byte header + 432 int8)\n"
		"  -q                print only the summary\n");
	exit(2);
}

int main(int argc, char **argv)
{
	const char *path = NULL, *fmt_s = NULL;
	enum fmt fmt = F_CF32;
	double rate = 0.0, shift = 0.0, shift_phase = 0.0;
	int swap = 0, cs16_api = 0, i, channels = 2;
	size_t block = 1024, skip = 0, max_in = 0, total_in = 0, n72 = 0;
	uint32_t flags = TETRA_RX_FLAG_BURST_EVENTS;
	tetra_rx_config_t cfg;
	tetra_rx_status_t st;
	tetra_rx_t *rx;
	resamp_t *rs = NULL;
	dqpsk_mod_t *mod = NULL;
	FILE *f;
	unsigned char hdr[12];
	float *in, *out;
	unsigned char *raw;
	size_t bytes_per = 8, out_cap;
	struct timespec t0, t1;
	double cpu = 0.0;

	for (i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "--format") && i + 1 < argc)
			fmt_s = argv[++i];
		else if (!strcmp(argv[i], "--rate") && i + 1 < argc)
			rate = atof(argv[++i]);
		else if (!strcmp(argv[i], "--swap-iq"))
			swap = 1;
		else if (!strcmp(argv[i], "--shift") && i + 1 < argc)
			shift = atof(argv[++i]);
		else if (!strcmp(argv[i], "--skip") && i + 1 < argc)
			skip = (size_t) atoll(argv[++i]);
		else if (!strcmp(argv[i], "--max") && i + 1 < argc)
			max_in = (size_t) atoll(argv[++i]);
		else if (!strcmp(argv[i], "--block") && i + 1 < argc)
			block = (size_t) atoll(argv[++i]);
		else if (!strcmp(argv[i], "--cs16-api"))
			cs16_api = 1;
		else if (!strcmp(argv[i], "--aach"))
			flags |= TETRA_RX_FLAG_AACH_EVENTS;
		else if (!strcmp(argv[i], "--blocks"))
			flags |= TETRA_RX_FLAG_BLOCK_EVENTS;
		else if (!strcmp(argv[i], "--no-traffic"))
			flags |= TETRA_RX_FLAG_NO_TRAFFIC;
		else if (!strcmp(argv[i], "--dc-block"))
			flags |= TETRA_RX_FLAG_DC_BLOCK;
		else if (!strcmp(argv[i], "--dump-bits") && i + 1 < argc)
			g.dump_bits = fopen(argv[++i], "wb");
		else if (!strcmp(argv[i], "--dump-soft") && i + 1 < argc)
			g.dump_soft = fopen(argv[++i], "wb");
		else if (!strcmp(argv[i], "-q"))
			g.quiet = 1;
		else if (argv[i][0] == '-')
			usage();
		else
			path = argv[i];
	}
	if (!path || block == 0)
		usage();
	f = fopen(path, "rb");
	if (!f) {
		perror(path);
		return 1;
	}

	if (fmt_s) {
		if (!strcmp(fmt_s, "cf32"))
			fmt = F_CF32;
		else if (!strcmp(fmt_s, "cs16"))
			fmt = F_CS16;
		else if (!strcmp(fmt_s, "cu8"))
			fmt = F_CU8;
		else if (!strcmp(fmt_s, "cs8"))
			fmt = F_CS8;
		else if (!strcmp(fmt_s, "bits"))
			fmt = F_BITS;
		else if (strcmp(fmt_s, "wav"))
			usage();
	}
	/* WAV container */
	if ((!fmt_s || !strcmp(fmt_s, "wav")) && fread(hdr, 1, 12, f) == 12 && !memcmp(hdr, "RIFF", 4) &&
	    !memcmp(hdr + 8, "WAVE", 4)) {
		unsigned char ch[8], fm[16];
		int have_fmt = 0;

		for (;;) {
			uint32_t len;

			if (fread(ch, 1, 8, f) != 8) {
				fprintf(stderr, "%s: no data chunk\n", path);
				return 1;
			}
			len = (uint32_t) ch[4] | ((uint32_t) ch[5] << 8) | ((uint32_t) ch[6] << 16) | ((uint32_t) ch[7] << 24);
			if (!memcmp(ch, "fmt ", 4) && len >= 16) {
				unsigned int tag, bits, wrate;

				if (fread(fm, 1, 16, f) != 16)
					return 1;
				fseek(f, (long) (len - 16 + (len & 1)), SEEK_CUR);
				tag = fm[0] | (fm[1] << 8);
				channels = fm[2] | (fm[3] << 8);
				wrate = (unsigned int) fm[4] | ((unsigned int) fm[5] << 8) | ((unsigned int) fm[6] << 16) |
					((unsigned int) fm[7] << 24);
				bits = fm[14] | (fm[15] << 8);
				if (channels != 2) {
					fprintf(stderr, "%s: need a 2 channel (I/Q) WAV\n", path);
					return 1;
				}
				if ((tag == 1 || tag == 0xfffe) && bits == 16)
					fmt = F_CS16;
				else if (tag == 1 && bits == 8)
					fmt = F_CU8;
				else if (tag == 3 && bits == 32)
					fmt = F_CF32;
				else {
					fprintf(stderr, "%s: unsupported WAV format %u/%u bit\n", path, tag, bits);
					return 1;
				}
				if (rate == 0.0)
					rate = wrate;
				have_fmt = 1;
			} else if (!memcmp(ch, "data", 4)) {
				if (!have_fmt) {
					fprintf(stderr, "%s: data before fmt\n", path);
					return 1;
				}
				break;
			} else
				fseek(f, (long) (len + (len & 1)), SEEK_CUR);
		}
	} else {
		if (fmt_s && !strcmp(fmt_s, "wav")) {
			fprintf(stderr, "%s: not a WAV file\n", path);
			return 1;
		}
		rewind(f);
	}
	if (rate == 0.0)
		rate = 72000.0;
	switch (fmt) {
	case F_CF32:
		bytes_per = 8;
		break;
	case F_CS16:
		bytes_per = 4;
		break;
	case F_CU8:
	case F_CS8:
		bytes_per = 2;
		break;
	case F_BITS:
		bytes_per = 2;      /* two bits = one symbol */
		rate = 72000.0;
		mod = dqpsk_mod_new(4);
		break;
	}
	if (rate != 72000.0)
		rs = resamp_new(rate, 72000.0, 14000.0);
	if (cs16_api && (fmt != F_CS16 || rs || swap || shift != 0.0)) {
		fprintf(stderr, "--cs16-api needs unmodified 72000 Hz cs16 input\n");
		return 1;
	}
	if (skip)
		fseek(f, (long) (skip * bytes_per), SEEK_CUR);

	memset(&cfg, 0, sizeof(cfg));
	cfg.event_cb = on_event;
	cfg.flags = flags;
	rx = tetra_rx_create(&cfg);
	if (!rx) {
		fprintf(stderr, "tetra_rx_create failed\n");
		return 1;
	}

	raw = malloc(block * bytes_per);
	in = malloc(block * 4 * 2 * sizeof(float));
	out_cap = (size_t) ((double) block * 4.0 * 72000.0 / rate) + 16;
	out = malloc(out_cap * 2 * sizeof(float));

	for (;;) {
		size_t n = fread(raw, bytes_per, block, f), k, n_out;
		const float *feed;

		if (n == 0)
			break;
		if (max_in && total_in + n > max_in)
			n = max_in - total_in;
		if (n == 0)
			break;
		total_in += n;

		if (cs16_api) {
			clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t0);
			tetra_rx_process_cs16(rx, (const int16_t *) raw, n);
			clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t1);
			cpu += (double) (t1.tv_sec - t0.tv_sec) + 1e-9 * (double) (t1.tv_nsec - t0.tv_nsec);
			n72 += n;
			continue;
		}
		switch (fmt) {
		case F_CF32:
			memcpy(in, raw, n * 8);
			break;
		case F_CS16:
			for (k = 0; k < 2 * n; k++)
				in[k] = (float) ((const int16_t *) raw)[k] * (1.0f / 32768.0f);
			break;
		case F_CU8:
			for (k = 0; k < 2 * n; k++)
				in[k] = ((float) raw[k] - 127.5f) * (1.0f / 128.0f);
			break;
		case F_CS8:
			for (k = 0; k < 2 * n; k++)
				in[k] = (float) ((const int8_t *) raw)[k] * (1.0f / 128.0f);
			break;
		case F_BITS:
			dqpsk_mod(mod, raw, 2 * n, in);
			n *= 4;
			break;
		}
		if (swap)
			for (k = 0; k < n; k++) {
				float t = in[2 * k];

				in[2 * k] = in[2 * k + 1];
				in[2 * k + 1] = t;
			}
		if (shift != 0.0)
			freq_shift(in, n, shift, rate, &shift_phase);
		if (rs) {
			n_out = resamp_process(rs, in, n, out);
			feed = out;
		} else {
			n_out = n;
			feed = in;
		}
		clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t0);
		tetra_rx_process_cf32(rx, feed, n_out);
		clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t1);
		cpu += (double) (t1.tv_sec - t0.tv_sec) + 1e-9 * (double) (t1.tv_nsec - t0.tv_nsec);
		n72 += n_out;
	}

	tetra_rx_get_status(rx, &st);
	printf("---- %s: %.2f s of signal (%zu samples at 72 kHz)\n", path, (double) n72 / 72000.0, n72);
	print_status(stdout, &st);
	printf("events: sync=%llu bsch=%llu sysinfo=%llu aach=%llu signal=%llu traffic=%llu "
	       "(enc? %u, clear %u, encrypted %u) hash=%016llx\n",
	       (unsigned long long) g.n_events[TETRA_RX_EV_SYNC], (unsigned long long) g.n_events[TETRA_RX_EV_BSCH],
	       (unsigned long long) g.n_events[TETRA_RX_EV_SYSINFO], (unsigned long long) g.n_events[TETRA_RX_EV_AACH],
	       (unsigned long long) g.n_events[TETRA_RX_EV_SIGNAL], (unsigned long long) g.n_events[TETRA_RX_EV_TRAFFIC],
	       g.tch_enc[0], g.tch_enc[1], g.tch_enc[2], (unsigned long long) g.hash);
	if (n72)
		printf("cpu: %.1f ns/sample, %.0f x realtime on this host\n", cpu * 1e9 / (double) n72,
		       (double) n72 / 72000.0 / cpu);

	tetra_rx_destroy(rx);
	if (g.dump_bits)
		fclose(g.dump_bits);
	if (g.dump_soft)
		fclose(g.dump_soft);
	free(raw);
	free(in);
	free(out);
	resamp_free(rs);
	dqpsk_mod_free(mod);
	fclose(f);
	return 0;
}
