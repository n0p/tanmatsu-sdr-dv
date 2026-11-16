/*
 * dmr_rx - static name tables.
 *
 * Opcode values and names follow ETSI TS 102 361-1/-2/-4; the manufacturer
 * specific ones (Motorola Capacity Plus / Connect Plus, Hytera XPT) follow
 * the naming commonly used by DSD-family decoders.
 *
 * SPDX-License-Identifier: AGPL-3.0-only
 */
#include "dmr_rx.h"

extern "C" {

const char *dmr_rx_sync_type_name(uint8_t t)
{
    switch (t) {
    case DMR_SYNC_BS_VOICE: return "BS voice";
    case DMR_SYNC_BS_DATA: return "BS data";
    case DMR_SYNC_MS_VOICE: return "MS voice";
    case DMR_SYNC_MS_DATA: return "MS data";
    case DMR_SYNC_DM_TS1_VOICE: return "DM TS1 voice";
    case DMR_SYNC_DM_TS1_DATA: return "DM TS1 data";
    case DMR_SYNC_DM_TS2_VOICE: return "DM TS2 voice";
    case DMR_SYNC_DM_TS2_DATA: return "DM TS2 data";
    default: return "none";
    }
}

const char *dmr_rx_data_type_name(uint8_t t)
{
    switch (t) {
    case DMR_DT_PI_HEADER: return "PI Header";
    case DMR_DT_VOICE_LC_HEADER: return "Voice LC Header";
    case DMR_DT_TERMINATOR_LC: return "Terminator with LC";
    case DMR_DT_CSBK: return "CSBK";
    case DMR_DT_MBC_HEADER: return "MBC Header";
    case DMR_DT_MBC_CONTINUATION: return "MBC Continuation";
    case DMR_DT_DATA_HEADER: return "Data Header";
    case DMR_DT_RATE_12_DATA: return "Rate 1/2 Data";
    case DMR_DT_RATE_34_DATA: return "Rate 3/4 Data";
    case DMR_DT_IDLE: return "Idle";
    case DMR_DT_RATE_1_DATA: return "Rate 1 Data";
    case DMR_DT_UNIFIED_SINGLE: return "Unified Single Block Data";
    default: return "Reserved";
    }
}

const char *dmr_rx_flco_name(uint8_t flco, uint8_t fid)
{
    if (fid == 0x00) {
        switch (flco) {
        case 0x00: return "Group Voice Channel User";
        case 0x03: return "Unit to Unit Voice Channel User";
        case 0x04: return "Talker Alias Header";
        case 0x05: return "Talker Alias Block 1";
        case 0x06: return "Talker Alias Block 2";
        case 0x07: return "Talker Alias Block 3";
        case 0x08: return "GPS Info";
        case 0x30: return "Terminator Data LC";
        default: break;
        }
    } else if (fid == 0x10) { /* Motorola */
        switch (flco) {
        case 0x00: return "Group Voice Channel User (Motorola)";
        case 0x03: return "Unit to Unit Voice Channel User (Motorola)";
        case 0x04: return "Capacity Plus Group Voice";
        case 0x07: return "Capacity Plus Private Voice";
        default: return "Motorola LC";
        }
    } else if (fid == 0x06) {
        return "Connect Plus LC";
    } else if (fid == 0x68 || fid == 0x08) {
        switch (flco) {
        case 0x00: return "Group Voice Channel User (Hytera)";
        case 0x03: return "Unit to Unit Voice Channel User (Hytera)";
        default: return "Hytera LC";
        }
    }
    return "Unknown LC";
}

const char *dmr_rx_csbk_name(uint8_t op, uint8_t fid)
{
    if (fid == 0x00) {
        switch (op) {
        /* Tier II (TS 102 361-2) */
        case 0x04: return "UU_V_Req (Unit to Unit Voice Service Request)";
        case 0x05: return "UU_Ans_Rsp (Unit to Unit Voice Service Answer Response)";
        case 0x07: return "CT_CSBK (Channel Timing)";
        case 0x26: return "NACK_Rsp / C_NACKD (Negative Acknowledge)";
        case 0x38: return "BS_Dwn_Act / TD_GRANT_MI (BS Outbound Activation)";
        case 0x3D: return "Pre_CSBK (Preamble)";
        /* Tier III (TS 102 361-4) */
        case 0x19: return "C_ALOHA (Aloha)";
        case 0x1A: return "C_UDTHD (Unified Data Transport Outbound Header)";
        case 0x1B: return "C_UDTHU (Unified Data Transport Inbound Header)";
        case 0x1C: return "C_AHOY (Ahoy)";
        case 0x1E: return "C_ACKVIT (Ack Vitation)";
        case 0x1F: return "C_RAND (Random Access Service Request)";
        case 0x20: return "C_ACKD (Ack Outbound TSCC)";
        case 0x21: return "C_ACKU (Ack Inbound TSCC)";
        case 0x22: return "P_ACKD (Ack Outbound Payload)";
        case 0x23: return "P_ACKU (Ack Inbound Payload)";
        case 0x27: return "C_NACKU (Nack Inbound TSCC)";
        case 0x28: return "C_BCAST (Announcement / Broadcast)";
        case 0x2A: return "P_MAINT (Maintenance)";
        case 0x2E: return "P_CLEAR (Clear)";
        case 0x2F: return "P_PROTECT (Protect)";
        case 0x30: return "PV_GRANT (Private Voice Channel Grant)";
        case 0x31: return "TV_GRANT (Talkgroup Voice Channel Grant)";
        case 0x32: return "BTV_GRANT (Broadcast Talkgroup Voice Channel Grant)";
        case 0x33: return "PD_GRANT (Private Data Channel Grant)";
        case 0x34: return "TD_GRANT (Talkgroup Data Channel Grant)";
        case 0x35: return "PV_GRANT_DX (Duplex Private Voice Channel Grant)";
        case 0x36: return "PD_GRANT_DX (Duplex Private Data Channel Grant)";
        case 0x37: return "PD_GRANT_MI (Private Data Channel Grant, Multi Item)";
        case 0x39: return "C_MOVE (Move TSCC)";
        default: return "Unknown CSBK";
        }
    }
    if (fid == 0x10) { /* Motorola */
        switch (op) {
        case 0x1F: return "Capacity Plus Call Alert";
        case 0x20: return "Capacity Plus Call Alert Ack";
        case 0x24: return "Motorola Extended Function";
        case 0x3A: return "Capacity Max Aloha / Sys Parms";
        case 0x3B: return "Capacity Plus Neighbours";
        case 0x3D: return "Capacity Plus Preamble";
        case 0x3E: return "Capacity Plus Channel Status";
        default: return "Motorola CSBK";
        }
    }
    if (fid == 0x06) { /* Motorola Connect Plus */
        switch (op) {
        case 0x01: return "Connect Plus Neighbours";
        case 0x03: return "Connect Plus Voice Channel Grant";
        case 0x06: return "Connect Plus Data Channel Grant";
        case 0x0C: return "Connect Plus Terminate Channel Grant";
        case 0x11: return "Connect Plus Registration Request";
        case 0x12: return "Connect Plus Registration Response";
        case 0x18: return "Connect Plus Talkgroup Affiliation";
        default: return "Connect Plus CSBK";
        }
    }
    if (fid == 0x68 || fid == 0x08) { /* Hytera */
        switch (op) {
        case 0x0A: return "Hytera XPT Site State";
        case 0x0B: return "Hytera XPT Adjacent Sites";
        case 0x28: return "Hytera Sync Broadcast";
        default: return "Hytera CSBK";
        }
    }
    return "Unknown CSBK (manufacturer FID)";
}

const char *dmr_rx_slco_name(uint8_t slco)
{
    switch (slco) {
    case 0x0: return "Nul_Msg";
    case 0x1: return "Act_Updt (Activity Update)";
    case 0x2: return "C_SYS_Parms (TSCC System Parameters)";
    case 0x3: return "P_SYS_Parms (Payload Channel System Parameters)";
    case 0x8: return "Hytera XPT";
    case 0x9: return "Connect Plus Voice Channel";
    case 0xA: return "Connect Plus Control Channel";
    case 0xF: return "Capacity Plus Rest Channel";
    default: return "Unknown SLCO";
    }
}

const char *dmr_rx_event_name(uint8_t e)
{
    switch (e) {
    case DMR_EVT_SYNC_ACQUIRED: return "SYNC_ACQUIRED";
    case DMR_EVT_SYNC_LOST: return "SYNC_LOST";
    case DMR_EVT_DATA_BURST: return "DATA_BURST";
    case DMR_EVT_VOICE_BURST: return "VOICE_BURST";
    case DMR_EVT_LC: return "LC";
    case DMR_EVT_CSBK: return "CSBK";
    case DMR_EVT_PI_HEADER: return "PI_HEADER";
    case DMR_EVT_DATA_HEADER: return "DATA_HEADER";
    case DMR_EVT_SHORT_LC: return "SHORT_LC";
    case DMR_EVT_TALKER_ALIAS: return "TALKER_ALIAS";
    case DMR_EVT_CALL_START: return "CALL_START";
    case DMR_EVT_CALL_END: return "CALL_END";
    case DMR_EVT_CALL_UPDATE: return "CALL_UPDATE";
    case DMR_EVT_DATA_PDU: return "DATA_PDU";
    default: return "?";
    }
}

} // extern "C"
