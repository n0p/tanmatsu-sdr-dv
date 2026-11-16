/* tetra_rx - public API glue
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "tetra_ctx.h"

#include <tetra_mle_pdu.h>

tetra_rx_t *tetra_rx_create(const tetra_rx_config_t *cfg)
{
	void *(*alloc)(size_t) = (cfg && cfg->alloc) ? cfg->alloc : malloc;
	struct tetra_rx *rx = alloc(sizeof(*rx));

	if (!rx)
		return NULL;
	memset(rx, 0, sizeof(*rx));
	if (cfg)
		rx->cfg = *cfg;
	if (!rx->cfg.alloc || !rx->cfg.free) {
		rx->cfg.alloc = malloc;
		rx->cfg.free = (cfg && cfg->alloc) ? cfg->free : free;
	}
	tetra_lmac_init(rx);
	tetra_modem_init(rx);
	tetra_umac_reset(rx);
	return rx;
}

void tetra_rx_destroy(tetra_rx_t *rx)
{
	if (rx && rx->cfg.free)
		rx->cfg.free(rx);
}

void tetra_rx_reset(tetra_rx_t *rx)
{
	tetra_modem_reset(rx);
	rx->cell_valid = 0;
	rx->time_valid = 0;
	memset(&rx->time, 0, sizeof(rx->time));
	memset(&rx->cur, 0, sizeof(rx->cur));
	memset(&rx->st, 0, sizeof(rx->st));
	tetra_umac_reset(rx);
}

void tetra_rx_process_cf32(tetra_rx_t *rx, const float *iq, size_t n_complex)
{
	tetra_modem_cf32(rx, iq, n_complex);
	rx->st.samples += n_complex;
}

void tetra_rx_process_cs16(tetra_rx_t *rx, const int16_t *iq, size_t n_complex)
{
	tetra_modem_cs16(rx, iq, n_complex);
	rx->st.samples += n_complex;
}

size_t tetra_rx_get_constellation(const tetra_rx_t *rx, float *iq, size_t max_symbols)
{
	const struct tetra_modem *m = &rx->m;
	size_t n = max_symbols, k;
	float pwr = 0.0f, g;

	if (n > TETRA_RX_CONSTELLATION_MAX)
		n = TETRA_RX_CONSTELLATION_MAX;
	if (n > m->nsym)
		n = m->nsym;
	for (k = 0; k < n; k++) {
		uint32_t i = (m->nsym - (uint32_t) n + (uint32_t) k) & TM_RING_MASK;

		iq[2 * k] = m->dr[i];
		iq[2 * k + 1] = m->di[i];
		pwr += m->dr[i] * m->dr[i] + m->di[i] * m->di[i];
	}
	g = pwr > 1e-30f ? sqrtf((float) n / pwr) : 0.0f;
	for (k = 0; k < 2 * n; k++)
		iq[k] *= g;
	return n;
}

void tetra_rx_get_status(const tetra_rx_t *rx, tetra_rx_status_t *st)
{
	const struct tetra_modem *m = &rx->m;

	*st = rx->st;
	st->locked = m->state == TM_ST_LOCKED;
	st->time.valid = rx->time_valid;
	st->freq_offset_hz = m->freq_est;
	st->timing_ppm = m->ppm;
	st->level_dbfs = m->level > 1e-20f ? 10.0f * log10f(m->level) : -200.0f;
	st->evm_snr_db = m->snr_lin > 1e-6f ? 10.0f * log10f(m->snr_lin) : -60.0f;
	st->train_ber = m->ber;
}

/* ---- events ---------------------------------------------------------------- */

tetra_rx_event_t *tetra_rx_ev_begin(struct tetra_rx *rx)
{
	memset(&rx->ev, 0, sizeof(rx->ev));
	return &rx->ev;
}

void tetra_rx_emit(struct tetra_rx *rx, tetra_rx_event_type_t type)
{
	rx->ev.type = type;
	rx->ev.time.valid = rx->time_valid;
	rx->ev.time.tn = (uint8_t) rx->time.tn;
	rx->ev.time.fn = (uint8_t) rx->time.fn;
	rx->ev.time.mn = (uint8_t) rx->time.mn;
	if (rx->cfg.event_cb)
		rx->cfg.event_cb(rx->cfg.user, &rx->ev);
}

/* ---- names ----------------------------------------------------------------- */

const char *tetra_rx_chan_name(uint8_t chan)
{
	static const char *const names[TETRA_RX_CH_COUNT] = {
		"BSCH", "SB2", "SCH/HD", "SCH/F", "STCH", "AACH", "TCH/S",
	};

	return chan < TETRA_RX_CH_COUNT ? names[chan] : "?";
}

const char *tetra_rx_pdisc_name(uint8_t pdisc)
{
	return tetra_get_mle_pdisc_name(pdisc);
}

const char *tetra_rx_version(void)
{
	return "tetra_rx 1.0 (osmo-tetra 59fb6039)";
}
