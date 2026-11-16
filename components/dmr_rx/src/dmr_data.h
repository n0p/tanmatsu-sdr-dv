/*
 * dmr_rx - packet data: the data block codings that carry no BPTC (rate 3/4
 * trellis, rate 1), the checksums of data blocks and packets, and the
 * interpretation of a reassembled packet (text messages).
 *
 * SPDX-License-Identifier: AGPL-3.0-only
 */
#ifndef DMR_RX_DATA_H
#define DMR_RX_DATA_H

#include <stddef.h>
#include <stdint.h>

#include "dmr_rx.h"

namespace dmrrx {

/* Rate 3/4 trellis code (TS 102 361-1 B.2.4 and annex B tables): 144 data
 * bits in the 196 info bits of a burst. */
struct Trellis {
    uint8_t data[18];
    uint8_t errs; /* Hamming distance between the 196 received bits and the
                     re-encoded decoder output                               */
};
void trellis34_decode(const uint8_t *burst, Trellis &out);
/* Writes the 196 info bits of the burst; the other bits are left alone. */
void trellis34_encode(const uint8_t data[18], uint8_t *burst);
/* Largest re-encode distance at which a rate 3/4 block is still taken as
 * received. The code has only 52 check bits: any 196 bits are within 8 to 16
 * bits of a code word (0.8 % of random blocks within 10), and a block with
 * more than a few errors decodes to the wrong code word, so this tells a
 * block from noise and not a right block from a wrong one. That is what the
 * CRC-9 of the block and the CRC-32 of the packet are for. */
enum { kTrellisPlausibleErrs = 10 };

/* Rate 1 (uncoded): 192 data bits. */
void rate1_decode(const uint8_t *burst, uint8_t data[24]);
void rate1_encode(const uint8_t data[24], uint8_t *burst);

/* Bytes in a data block of this data type (rate 1/2, 3/4, 1), 0 for any
 * other type. */
unsigned data_block_len(uint8_t data_type);

/* CRC-9 of a confirmed data block (TS 102 361-1 B.3.10): the value that is
 * transmitted in a block of this data type with this serial number and
 * payload. */
uint16_t data_crc9(uint8_t data_type, uint8_t dbsn, const uint8_t *payload, size_t n);
/* CRC-32 of a packet (B.3.9): pdu is everything the blocks carry including
 * the four CRC octets at the end. data_crc32() computes what those four
 * octets must be (as the 32 bit value of the octets in transmission order). */
uint32_t data_crc32(const uint8_t *pdu, size_t n_without_crc);
bool data_crc32_ok(const uint8_t *pdu, size_t n);

/* Fill kind, format, text, status and the UDP ports of p from its header
 * fields and data / len. text: cap bytes of storage for the text. */
void pdu_interpret(dmr_evt_data_pdu_t &p, char *text, size_t cap);

} // namespace dmrrx

#endif
