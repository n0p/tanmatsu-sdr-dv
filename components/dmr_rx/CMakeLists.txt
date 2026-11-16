# dmr_rx - DMR (ETSI TS 102 361) receiver / decoder, ESP-IDF component.
#
# No dependencies on other IDF components (libc / libm only).
#
# Stage 2, the AMBE+2 speech decoder (8 kHz PCM), is built in by default. To
# leave it out set DMR_RX_WITH_VOICE to OFF before the components are
# registered, e.g. `idf.py -DDMR_RX_WITH_VOICE=OFF build` or a set() in the
# project CMakeLists.txt. dmr_rx_has_voice() tells at run time.

if(NOT DEFINED DMR_RX_WITH_VOICE)
    set(DMR_RX_WITH_VOICE ON)
endif()

idf_component_register(
    SRCS
        "src/dmr_rx.cpp"
        "src/dmr_data.cpp"
        "src/dmr_demod.cpp"
        "src/dmr_pdu.cpp"
        "src/dmr_fec.cpp"
        "src/dmr_names.cpp"
        "src/dmr_voice.cpp"
        "third_party/mmdvmhost/BPTC19696.cpp"
        "third_party/mmdvmhost/CRC.cpp"
        "third_party/mmdvmhost/DMREMB.cpp"
        "third_party/mmdvmhost/DMREmbeddedData.cpp"
        "third_party/mmdvmhost/DMRFullLC.cpp"
        "third_party/mmdvmhost/DMRLC.cpp"
        "third_party/mmdvmhost/DMRShortLC.cpp"
        "third_party/mmdvmhost/DMRSlotType.cpp"
        "third_party/mmdvmhost/Golay2087.cpp"
        "third_party/mmdvmhost/Hamming.cpp"
        "third_party/mmdvmhost/QR1676.cpp"
        "third_party/mmdvmhost/RS129.cpp"
        # AMBE FEC (always needed) and, inside #ifdef DMR_RX_WITH_VOICE, the
        # speech decoder
        "third_party/mbelib/ecc.c"
        "third_party/mbelib/ambe3600x2450_ecc.c"
        "third_party/mbelib/ambe3600x2450.c"
        "third_party/mbelib/mbelib.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src" "third_party/mmdvmhost" "third_party/mbelib" "third_party/dsd"
)

if(DMR_RX_WITH_VOICE)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE DMR_RX_WITH_VOICE)
endif()

# The code is C11 / C++17 and float only: -Wdouble-promotion keeps it that
# way. -O2 because this is the signal path (the project default may be -Og or
# -Os).
target_compile_options(${COMPONENT_LIB} PRIVATE
    -O2 -Wdouble-promotion
    "$<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17>"
    "$<$<COMPILE_LANGUAGE:C>:-std=gnu11>")
