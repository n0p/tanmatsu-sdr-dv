idf_component_register(
	SRCS
		"main.c"
		"radio.c"
		"dsp.c"
		"capture.c"
		"audio.c"
		"console.c"
		"decoder.c"
		"memory.c"
		"ui.c"
		"recfmt.c"
		"rec.c"
		"leds.c"
		"battery.c"
		"msglog.c"
		"shot.c"
	PRIV_REQUIRES
		esp_lcd
		fatfs
		wear_levelling
		nvs_flash
		badge-bsp
		esp_rtl_sdr
		dmr_rx
		tetra_rx
		tetra_codec
		esp_timer
		esp_driver_i2s
		esp_driver_gpio
		esp_driver_usb_serial_jtag
		esp_driver_sdmmc
		sdmmc
		esp_hw_support
		opus
	INCLUDE_DIRS
		"."
)

idf_build_set_property(COMPILE_OPTIONS "-Wno-error=unused-variable" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-error=unused-const-variable" APPEND)

# The decoder panel lines are fixed-width and meant to be cut off at the panel edge.
set_source_files_properties(decoder.c PROPERTIES COMPILE_OPTIONS "-Wno-format-truncation")
# File names and notices: the buffers are sized for the values that occur, not for INT_MIN.
set_source_files_properties(rec.c console.c msglog.c shot.c PROPERTIES COMPILE_OPTIONS "-Wno-format-truncation")
