/*
 * dmr_rx - what a reassembled data packet says: status values, the short
 * data formats of the standard, and the text message formats of the radio
 * manufacturers that travel as UDP/IP (TS 102 361-1 clause 9.2, 102 361-3
 * clause 7; the manufacturer formats as they are publicly documented).
 *
 * A receiver does not know which format the two radios agreed on, so a
 * packet that fits none of the known ones exactly is searched for something
 * that reads like text.
 *
 * SPDX-License-Identifier: AGPL-3.0-only
 */
#include "dmr_data.h"

#include <string.h>

namespace dmrrx {

/* ------------------------------------------------------------------------ */
/* Text output: UTF-8, no control characters but '\n' and '\t'              */
/* ------------------------------------------------------------------------ */

struct Text {
    char *buf;
    size_t n, cap;
    bool full; /* cut off */

    void clear()
    {
        n    = 0;
        full = false;
    }
    void put(uint32_t c)
    {
        if (full || c == '\r' || c == 0xFEFFu)
            return;
        if ((c < 0x20u && c != '\n' && c != '\t') || (c >= 0x7Fu && c < 0xA0u))
            c = ' ';
        if (c > 0x10FFFFu || (c >= 0xD800u && c <= 0xDFFFu))
            c = 0xFFFDu;
        const size_t len = c < 0x80u ? 1u : c < 0x800u ? 2u : c < 0x10000u ? 3u : 4u;
        if (n + len >= cap) {
            full = true;
            return;
        }
        char *o = buf + n;
        if (len == 1) {
            o[0] = (char)c;
        } else if (len == 2) {
            o[0] = (char)(0xC0u | (c >> 6));
            o[1] = (char)(0x80u | (c & 0x3Fu));
        } else if (len == 3) {
            o[0] = (char)(0xE0u | (c >> 12));
            o[1] = (char)(0x80u | ((c >> 6) & 0x3Fu));
            o[2] = (char)(0x80u | (c & 0x3Fu));
        } else {
            o[0] = (char)(0xF0u | (c >> 18));
            o[1] = (char)(0x80u | ((c >> 12) & 0x3Fu));
            o[2] = (char)(0x80u | ((c >> 6) & 0x3Fu));
            o[3] = (char)(0x80u | (c & 0x3Fu));
        }
        n += len;
    }
    /* Terminate; without the white space around the text. False if nothing
     * is left. */
    bool finish()
    {
        size_t end = n;
        while (end && (buf[end - 1] == ' ' || buf[end - 1] == '\n' || buf[end - 1] == '\t'))
            end--;
        size_t start = 0;
        while (start < end && (buf[start] == ' ' || buf[start] == '\n' || buf[start] == '\t'))
            start++;
        memmove(buf, buf + start, end - start);
        n      = end - start;
        buf[n] = 0;
        return n != 0;
    }
};

static bool text_result(dmr_evt_data_pdu_t &p, Text &t, const char *format)
{
    if (!t.finish())
        return false;
    p.kind   = DMR_PDU_TEXT;
    p.format = format;
    return true;
}

static inline unsigned unit16(const uint8_t *d, bool be)
{
    return be ? ((unsigned)d[0] << 8) | d[1] : ((unsigned)d[1] << 8) | d[0];
}

/* n bytes of UTF-16 up to the first NUL. Returns the number of code units
 * taken; odd counts what cannot be part of a message (control characters,
 * broken surrogate pairs). */
static unsigned put_utf16(Text &t, const uint8_t *d, size_t n, bool be, unsigned &odd)
{
    unsigned units = 0;
    odd            = 0;
    for (size_t i = 0; i + 1 < n; i += 2) {
        uint32_t u = unit16(d + i, be);
        if (u == 0)
            break;
        units++;
        if (u >= 0xD800u && u <= 0xDBFFu && i + 3 < n) {
            const uint32_t v = unit16(d + i + 2, be);
            if (v >= 0xDC00u && v <= 0xDFFFu) {
                u = 0x10000u + ((u - 0xD800u) << 10) + (v - 0xDC00u);
                i += 2;
                units++;
            } else {
                odd++;
            }
        } else if ((u >= 0xD800u && u <= 0xDFFFu) || u >= 0xFFFEu || (u < 0x20u && u != '\n' && u != '\r' && u != '\t')) {
            odd++;
        }
        t.put(u);
    }
    return units;
}

/* Text of a format that says where it is and how it is coded. */
static bool text_utf16(dmr_evt_data_pdu_t &p, Text &t, const uint8_t *d, size_t n, bool be, const char *format)
{
    unsigned odd;
    t.clear();
    const unsigned units = put_utf16(t, d, n, be, odd);
    if (units == 0 || odd * 4u > units)
        return false;
    return text_result(p, t, format);
}

/* Bytes that are UTF-8 or (latin1) ISO 8859-1, up to the first NUL. */
static bool text_bytes(dmr_evt_data_pdu_t &p, Text &t, const uint8_t *d, size_t n, bool latin1, const char *format)
{
    t.clear();
    for (size_t i = 0; i < n && d[i]; i++) {
        uint32_t c = d[i];
        if (c >= 0x80u && !latin1) {
            const unsigned more = c >= 0xF0u ? 3u : c >= 0xE0u ? 2u : c >= 0xC0u ? 1u : 0u;
            if (more == 0 || i + more >= n) {
                c = 0xFFFDu;
            } else {
                c &= 0x3Fu >> more;
                for (unsigned k = 1; k <= more; k++) {
                    if ((d[i + k] & 0xC0u) != 0x80u) {
                        c = 0xFFFDu;
                        break;
                    }
                    c = (c << 6) | (d[i + k] & 0x3Fu);
                }
                if (c != 0xFFFDu)
                    i += more;
            }
        }
        t.put(c);
    }
    return text_result(p, t, format);
}

/* ------------------------------------------------------------------------ */
/* Looking for text in a packet of unknown format                           */
/* ------------------------------------------------------------------------ */

/* A UTF-16 code unit that messages are made of: the Latin, Greek and
 * Cyrillic alphabets, line ends, typographic punctuation, the Euro sign. */
static bool text_unit(unsigned u)
{
    return (u >= 0x20u && u <= 0x7Eu) || (u >= 0xA0u && u <= 0x52Fu) || u == '\n' || u == '\r' || u == '\t' ||
           (u >= 0x2010u && u <= 0x2027u) || u == 0x20ACu;
}

/* Bytes of the character at d if it is one of those in ASCII or UTF-8, else
 * 0. */
static size_t text_char8(const uint8_t *d, size_t left)
{
    const unsigned c = d[0];
    if (c < 0x80u)
        return text_unit(c) ? 1u : 0u;
    if (c >= 0xC2u && c <= 0xDFu && left >= 2 && (d[1] & 0xC0u) == 0x80u)
        return text_unit(((c & 0x1Fu) << 6) | (d[1] & 0x3Fu)) ? 2u : 0u;
    if (c >= 0xE0u && c <= 0xEFu && left >= 3 && (d[1] & 0xC0u) == 0x80u && (d[2] & 0xC0u) == 0x80u) {
        const unsigned u = ((c & 0x0Fu) << 12) | ((d[1] & 0x3Fu) << 6) | (d[2] & 0x3Fu);
        return u >= 0x800u && text_unit(u) ? 3u : 0u;
    }
    return 0;
}

static bool guess_text(dmr_evt_data_pdu_t &p, Text &t, const uint8_t *d, size_t n)
{
    /* the longest run of such code units, in either byte order, at an even
     * or an odd offset; of equally long ones the first */
    size_t best_off = 0, best_units = 0;
    bool best_be = false;
    for (int be = 0; be < 2; be++) {
        for (size_t al = 0; al < 2; al++) {
            size_t off = al, run = 0;
            for (size_t i = al;; i += 2) {
                const bool ok = i + 1 < n && text_unit(unit16(d + i, be != 0));
                if (ok) {
                    if (run == 0)
                        off = i;
                    run++;
                } else {
                    if (run > best_units || (run == best_units && run != 0 && off < best_off)) {
                        best_units = run;
                        best_off   = off;
                        best_be    = be != 0;
                    }
                    run = 0;
                    if (i + 1 >= n)
                        break;
                }
            }
        }
    }
    /* A field in front of the text or behind it can look like one more
     * character from some other alphabet: not part of the text if no other
     * character of the run is from the same block of 256. */
    for (int pass = 0; pass < 2 && best_units > 3; pass++) {
        const size_t at  = pass == 0 ? best_off : best_off + 2 * (best_units - 1);
        const unsigned u = unit16(d + at, best_be);
        if (u < 0x100u)
            continue;
        bool alone = true;
        for (size_t k = 0; k < best_units && alone; k++) {
            const size_t o = best_off + 2 * k;
            if (o != at && (unit16(d + o, best_be) >> 8) == (u >> 8))
                alone = false;
        }
        if (alone) {
            best_units--;
            if (pass == 0)
                best_off += 2;
        }
    }
    if (best_units >= 3) {
        /* binary data does not stay within two blocks of 256 for long, and
         * does not contain many ASCII characters */
        unsigned hi[2] = {0, 0}, n_hi = 0, ascii = 0;
        bool fits = true;
        for (size_t k = 0; k < best_units; k++) {
            const unsigned u = unit16(d + best_off + 2 * k, best_be);
            if (u < 0x80u) {
                ascii++;
                continue;
            }
            const unsigned h = u >> 8;
            if (n_hi > 0 && hi[0] == h)
                continue;
            if (n_hi > 1 && hi[1] == h)
                continue;
            if (n_hi == 2) {
                fits = false;
                break;
            }
            hi[n_hi++] = h;
        }
        if (fits && (ascii * 2u >= best_units || best_units >= 4)) {
            unsigned odd;
            t.clear();
            put_utf16(t, d + best_off, 2 * best_units, best_be, odd);
            if (text_result(p, t, best_be ? "text? (UTF-16BE)" : "text? (UTF-16LE)"))
                return true;
        }
    }

    /* 8 bit text, ASCII or UTF-8: most of the packet, zeros at the end
     * aside */
    while (n && d[n - 1] == 0)
        n--;
    size_t best = 0, at = 0, run = 0;
    for (size_t i = 0;;) {
        const size_t len = i < n ? text_char8(d + i, n - i) : 0;
        if (len) {
            run += len;
            i += len;
            continue;
        }
        if (run > best) {
            best = run;
            at   = i - run;
        }
        run = 0;
        if (i >= n)
            break;
        i++;
    }
    if (best >= 4 && best * 10u >= n * 7u) {
        unsigned ascii = 0;
        for (size_t i = 0; i < best; i++)
            ascii += d[at + i] < 0x80u;
        if (ascii * 2u >= best && text_bytes(p, t, d + at, best, false, "text? (8 bit)"))
            return true;
    }
    t.clear();
    t.finish();
    return false;
}

/* ------------------------------------------------------------------------ */
/* Manufacturer formats                                                     */
/* ------------------------------------------------------------------------ */

/* Motorola text messaging service (UDP port 4007):
 *   length (2, of what follows) | first header | address length | address |
 *   more headers while bit 7 of the one before is set | text UTF-16LE
 * first header: 0x80 more headers follow, 0x40 acknowledgement wanted,
 * 0x10 control message, 0x0F type (0 text, 15 acknowledgement). */
static bool tms(dmr_evt_data_pdu_t &p, Text &t, const uint8_t *d, size_t n)
{
    if (n < 4)
        return false;
    const size_t end = (((size_t)d[0] << 8) | d[1]) + 2u;
    if (end < 4 || end > n)
        return false;
    for (size_t i = end; i < n; i++)
        if (d[i] != 0)
            return false;
    const uint8_t first = d[2];
    size_t pos          = 4u + d[3];
    if (pos > end)
        return false;
    if (first & 0x80u) {
        for (unsigned k = 0;; k++) {
            if (pos >= end || k >= 4)
                return false;
            if (!(d[pos++] & 0x80u))
                break;
        }
    }
    const unsigned type = first & 0x0Fu;
    if ((first & 0x10u) || type != 0) {
        if (pos != end)
            return false;
        p.kind   = DMR_PDU_SERVICE;
        p.format = type == 0x0Fu ? "Motorola TMS acknowledgement" : "Motorola TMS control";
        return true;
    }
    return text_utf16(p, t, d + pos, end - pos, false, "Motorola TMS");
}

/* Hytera text message protocol (UDP port 5016):
 *   0x09 (0x89 reliable) | flags | opcode | length (2) | [option length (2)] |
 *   request id (4) | destination (4) | source (4) | text UTF-16LE | [option] |
 *   checksum | 0x03
 * length counts from behind itself to before the checksum; flags 0x40: with
 * option field (its content is the end of what the length counts). */
static bool tmp(dmr_evt_data_pdu_t &p, Text &t, const uint8_t *d, size_t n)
{
    if (n < 7 || (d[0] & 0x7Fu) != 0x09u || (d[1] & 0x3Fu) != 0)
        return false;
    const bool opt   = (d[1] & 0x40u) != 0;
    const size_t len = ((size_t)d[3] << 8) | d[4];
    const size_t pos = opt ? 7u : 5u;
    size_t end       = 5u + len;
    if (end < pos || end + 2u > n || d[end + 1] != 0x03u)
        return false;
    if (opt) {
        const size_t ol = ((size_t)d[5] << 8) | d[6];
        if (pos + ol > end)
            return false;
        end -= ol;
    }
    switch (d[2]) {
    case 0xA1: /* private message */
    case 0xB1: /* group message */
        if (end < pos + 12u)
            return false;
        return text_utf16(p, t, d + pos + 12, end - pos - 12, false, "Hytera TMP");
    case 0xA2:
    case 0xB2:
        p.kind   = DMR_PDU_SERVICE;
        p.format = "Hytera TMP acknowledgement";
        return true;
    case 0xAE:
    case 0xAF:
    case 0xBE:
    case 0xBF:
        p.kind   = DMR_PDU_SERVICE;
        p.format = "Hytera short data";
        return true;
    default:
        return false;
    }
}

/* Services that are known by their UDP port and carry no text. */
static const char *service_name(unsigned port)
{
    switch (port) {
    case 4001: return "location (LRRP)";
    case 4005: return "registration (ARS)";
    case 4008: return "telemetry";
    case 4009: return "over the air programming";
    case 4012: return "battery management";
    case 4013: return "job ticket";
    case 5017: return "location (Hytera)";
    default: return NULL;
    }
}

/* The payload of a UDP datagram. */
static void udp_payload(dmr_evt_data_pdu_t &p, Text &t, const uint8_t *d, size_t n)
{
    const char *svc = service_name(p.dst_port);
    if (!svc)
        svc = service_name(p.src_port);
    if (svc) {
        p.kind   = DMR_PDU_SERVICE;
        p.format = svc;
        return;
    }
    if (tms(p, t, d, n) || tmp(p, t, d, n))
        return;
    if (p.dst_port == 5016 || p.src_port == 5016) {
        /* the text message of the standard's port number: UTF-16BE */
        unsigned units = 0, good = 0;
        for (size_t i = 0; i + 1 < n; i += 2) {
            const unsigned u = unit16(d + i, true);
            if (u == 0)
                break;
            units++;
            good += text_unit(u);
        }
        if (units && good == units && text_utf16(p, t, d, n, true, "text message (UDP 5016)"))
            return;
    }
    if (guess_text(p, t, d, n))
        return;
    p.format = "UDP";
}

/* IPv4 (SAP 4). */
static bool ipv4(dmr_evt_data_pdu_t &p, Text &t, const uint8_t *d, size_t n)
{
    if (n < 20 || (d[0] >> 4) != 4)
        return false;
    const size_t ihl = (size_t)(d[0] & 0x0Fu) * 4u;
    size_t total     = ((size_t)d[2] << 8) | d[3];
    if (ihl < 20 || total < ihl)
        return false;
    if (total > n)
        total = n; /* blocks missing at the end */
    if (ihl > total)
        return false;
    if ((((unsigned)d[6] & 0x1Fu) << 8 | d[7]) != 0) {
        p.format = "IP fragment";
        return true;
    }
    switch (d[9]) {
    case 1:
        p.kind   = DMR_PDU_SERVICE;
        p.format = "ICMP";
        return true;
    case 6:
        p.format = "TCP";
        return true;
    case 17: break;
    default:
        p.format = "IP";
        return true;
    }
    const uint8_t *u = d + ihl;
    if (total < ihl + 8u) {
        p.format = "UDP";
        return true;
    }
    p.src_port  = (uint16_t)((u[0] << 8) | u[1]);
    p.dst_port  = (uint16_t)((u[2] << 8) | u[3]);
    size_t ulen = ((size_t)u[4] << 8) | u[5];
    if (ulen < 8 || ulen > total - ihl)
        ulen = total - ihl;
    udp_payload(p, t, u + 8, ulen - 8u);
    return true;
}

/* UDP/IPv4 with compressed header (SAP 3, TS 102 361-3 7.2):
 *   IP identification (2) | source / destination address id (1) |
 *   source port id (1, 7 bits) | destination port id (1, 7 bits) |
 *   the port numbers (2 each) whose id is 0 | payload */
static bool udp_compressed(dmr_evt_data_pdu_t &p, Text &t, const uint8_t *d, size_t n)
{
    if (n < 5)
        return false;
    const unsigned spid = d[3] & 0x7Fu, dpid = d[4] & 0x7Fu;
    size_t pos = 5;
    if (n < pos + (spid == 0 ? 2u : 0u) + (dpid == 0 ? 2u : 0u))
        return false;
    static const uint16_t kPort[3] = {0, 5016, 5017};
    if (spid == 0) {
        p.src_port = (uint16_t)((d[pos] << 8) | d[pos + 1]);
        pos += 2;
    } else if (spid < 3) {
        p.src_port = kPort[spid];
    }
    if (dpid == 0) {
        p.dst_port = (uint16_t)((d[pos] << 8) | d[pos + 1]);
        pos += 2;
    } else if (dpid < 3) {
        p.dst_port = kPort[dpid];
    }
    udp_payload(p, t, d + pos, n - pos);
    return true;
}

/* ------------------------------------------------------------------------ */
/* Short data with a defined format (TS 102 361-1 9.3.x, DD format)          */
/* ------------------------------------------------------------------------ */

static bool short_data(dmr_evt_data_pdu_t &p, Text &t, const uint8_t *d, size_t n)
{
    const unsigned dd = p.header[8] >> 2;
    t.clear();
    if (dd == 1) { /* BCD */
        static const char kBcd[] = "0123456789*#abc";
        for (size_t i = 0; i < 2 * n; i++) {
            const unsigned v = (d[i / 2] >> ((i & 1u) ? 0 : 4)) & 0x0Fu;
            if (v == 0x0Fu)
                break;
            t.put((uint8_t)kBcd[v]);
        }
        return text_result(p, t, "short data, BCD");
    }
    if (dd == 2) { /* 7 bit characters, packed */
        for (size_t i = 0; i + 7 <= 8 * n; i += 7) {
            unsigned c = 0;
            for (unsigned k = 0; k < 7; k++)
                c = (c << 1) | ((d[(i + k) >> 3] >> (7u - ((i + k) & 7u))) & 1u);
            if (c == 0)
                break;
            t.put(c);
        }
        return text_result(p, t, "short data, 7 bit");
    }
    if (dd >= 3 && dd <= 0x11) /* ISO 8859-1 ... -16: all read as -1 */
        return text_bytes(p, t, d, n, true, "short data, ISO 8859");
    if (dd == 0x12)
        return text_bytes(p, t, d, n, false, "short data, UTF-8");
    if (dd >= 0x13 && dd <= 0x15) {
        bool be = dd != 0x15;
        if (dd == 0x13 && n >= 2 && d[0] == 0xFFu && d[1] == 0xFEu)
            be = false;
        return text_utf16(p, t, d, n, be, "short data, UTF-16");
    }
    if (dd >= 0x16 && dd <= 0x18) {
        bool be = dd != 0x18;
        if (dd == 0x16 && n >= 4 && d[0] == 0xFFu && d[1] == 0xFEu)
            be = false;
        for (size_t i = 0; i + 3 < n; i += 4) {
            const uint32_t c = be ? ((uint32_t)d[i] << 24) | ((uint32_t)d[i + 1] << 16) | ((uint32_t)d[i + 2] << 8) | d[i + 3]
                                  : ((uint32_t)d[i + 3] << 24) | ((uint32_t)d[i + 2] << 16) | ((uint32_t)d[i + 1] << 8) | d[i];
            if (c == 0)
                break;
            t.put(c);
        }
        return text_result(p, t, "short data, UTF-32");
    }
    return false;
}

/* ------------------------------------------------------------------------ */

void pdu_interpret(dmr_evt_data_pdu_t &p, char *text, size_t cap)
{
    Text t   = {text, 0, cap, false};
    text[0]  = 0;
    p.text   = text;
    p.kind   = DMR_PDU_UNKNOWN;
    p.format = "data";
    p.status = 0;
    p.src_port = p.dst_port = 0;

    if (p.dpf == 14 && p.blocks == 0) {
        p.kind   = DMR_PDU_STATUS;
        p.format = "status";
        p.status = (uint16_t)(((p.header[8] & 3u) << 8) | p.header[9]);
        return;
    }
    const uint8_t *d = p.data;
    const size_t n   = p.len;
    if (n == 0)
        return;

    if (p.dpf == 13)
        short_data(p, t, d, n);
    if (p.kind == DMR_PDU_UNKNOWN && p.sap == 4)
        ipv4(p, t, d, n);
    else if (p.kind == DMR_PDU_UNKNOWN && p.sap == 3)
        udp_compressed(p, t, d, n);
    /* not what the header says, or nothing the header says anything about */
    if (p.kind == DMR_PDU_UNKNOWN && !tms(p, t, d, n) && !tmp(p, t, d, n))
        guess_text(p, t, d, n);
    if (p.kind != DMR_PDU_TEXT)
        text[0] = 0;
}

} // namespace dmrrx
