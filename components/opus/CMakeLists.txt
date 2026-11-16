# Opus encoder for the audio recorder: libopus 1.5.2 (https://opus-codec.org, BSD licence, see
# libopus/COPYING), the fixed-point build without the float API. Only the files the encoder needs
# are here (libopus/sources.txt). One change: celt/arch.h lets the build set OPUS_FAST_INT64, which
# upstream switches on for 64-bit CPUs only. The P4 multiplies 32 x 32 -> 64 in two instructions,
# and the encoder is 3 % faster with it.
file(STRINGS "${CMAKE_CURRENT_LIST_DIR}/libopus/sources.txt" opus_srcs)
list(TRANSFORM opus_srcs PREPEND "libopus/")

idf_component_register(
	SRCS ${opus_srcs}
	INCLUDE_DIRS "libopus/include"
	PRIV_INCLUDE_DIRS "libopus" "libopus/celt" "libopus/silk" "libopus/silk/fixed" "libopus/src"
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE
	OPUS_BUILD
	FIXED_POINT=1
	DISABLE_FLOAT_API
	VAR_ARRAYS
	HAVE_LRINT
	HAVE_LRINTF
	PACKAGE_VERSION="1.5.2"
	OPUS_FAST_INT64=1
)
# Third-party code: its warnings are not ours to fix.
target_compile_options(${COMPONENT_LIB} PRIVATE -w -O2)
