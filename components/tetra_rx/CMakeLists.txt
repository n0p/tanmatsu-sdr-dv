# SPDX-License-Identifier: AGPL-3.0-or-later
# ESP-IDF component: TETRA downlink receiver (own modem, channel decoder / PDU parser from osmo-tetra).

set(OT "third_party/osmo-tetra")

idf_component_register(
    SRCS
        "src/tetra_rx.c"
        "src/tetra_modem.c"
        "src/tetra_lmac.c"
        "src/tetra_umac.c"
        "src/tetra_cmce.c"
        "src/tetra_tch.c"
        "src/tetra_viterbi.c"
        "${OT}/src/lower_mac/crc_simple.c"
        "${OT}/src/lower_mac/tch_reordering.c"
        "${OT}/src/lower_mac/tetra_conv_enc.c"
        "${OT}/src/lower_mac/tetra_interleave.c"
        "${OT}/src/lower_mac/tetra_rm3014.c"
        "${OT}/src/lower_mac/tetra_scramb.c"
        "${OT}/src/phy/tetra_burst.c"
        "${OT}/src/tetra_cmce_pdu.c"
        "${OT}/src/tetra_common.c"
        "${OT}/src/tetra_llc_pdu.c"
        "${OT}/src/tetra_mac_pdu.c"
        "${OT}/src/tetra_mle_pdu.c"
        "${OT}/src/tetra_mm_pdu.c"
        "${OT}/src/tetra_sndcp_pdu.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src" "${OT}/compat" "${OT}/src"
)

target_compile_options(${COMPONENT_LIB} PRIVATE -O2 -Wdouble-promotion)

# The vendored osmo-tetra sources print uint32_t with %u; on this target uint32_t is "long unsigned"
# (same width), which trips -Werror=format.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-format)
