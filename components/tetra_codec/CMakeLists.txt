# TETRA speech decoder (ACELP, ETSI EN 300 395-2) -- opt-in.
#
# The codec is the ETSI reference C code. It is not part of this repository and may not be
# redistributed, so it is taken at build time from the archive ETSI publishes with the standard,
# en_30039502v010301p0.zip. Put that file in the project root (it is git-ignored), or point
# TETRA_CODEC_ZIP (CMake or environment variable) at it. Without it the component builds as a
# stub: tetra_codec_available() returns false and the app shows calls without playing them.
idf_component_register(SRCS "src/tetra_codec.c" INCLUDE_DIRS "include")

if(CMAKE_BUILD_EARLY_EXPANSION)
	return()
endif()

set(zip "")
if(DEFINED TETRA_CODEC_ZIP)
	set(zip "${TETRA_CODEC_ZIP}")
elseif(DEFINED ENV{TETRA_CODEC_ZIP})
	set(zip "$ENV{TETRA_CODEC_ZIP}")
elseif(EXISTS "${CMAKE_SOURCE_DIR}/en_30039502v010301p0.zip")
	set(zip "${CMAKE_SOURCE_DIR}/en_30039502v010301p0.zip")
endif()

if(zip STREQUAL "")
	message(STATUS "tetra_codec: no ETSI codec archive, TETRA voice is not built in")
	return()
endif()
if(NOT EXISTS "${zip}")
	message(FATAL_ERROR "tetra_codec: TETRA_CODEC_ZIP=${zip} does not exist")
endif()

# Unpack the C sources into the build directory (never into the source tree).
set(etsi_dir "${CMAKE_CURRENT_BINARY_DIR}/etsi")
idf_build_get_property(python PYTHON)
execute_process(
	COMMAND ${python} "${COMPONENT_DIR}/prepare.py" "${zip}" "${etsi_dir}"
	RESULT_VARIABLE res
)
if(NOT res EQUAL 0)
	message(FATAL_ERROR "tetra_codec: could not unpack ${zip}")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${zip}" "${COMPONENT_DIR}/prepare.py")

target_compile_definitions(${COMPONENT_LIB} PRIVATE TETRA_CODEC_ETSI)
target_include_directories(${COMPONENT_LIB} PRIVATE "${etsi_dir}")
# The reference code is one program spread over several files with global state and generic names
# (L, p, add, round ...). src/tetra_codec.c includes all of it into one translation unit, and
# -fwhole-program makes everything except the three API functions local to it. Its warnings are
# not ours to fix: -w.
target_compile_options(${COMPONENT_LIB} PRIVATE -fwhole-program -w -O2)
message(STATUS "tetra_codec: TETRA voice built in from ${zip} (binary is not redistributable)")
